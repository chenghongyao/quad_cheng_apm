#include "AttitudeControl.h"
#include "AP_Motor.h"
#include "AC_PID.h"
#include "paramter.h"
#include "sys.h"


float rate_bf_to_motor_roll(float rate_target_cds)
{
	float p,i,d;            // used to capture pid values for logging
	float current_rate;     // this iteration's rate
	float rate_error;       // simply target_rate - current_rate
	
	// get current rate
  // To-Do: make getting gyro rates more efficient?
	current_rate = (ahrs.omega.x * AC_ATTITUDE_CONTROL_DEGX100);		//期望速率
	// calculate error and call pid controller
	rate_error = rate_target_cds - current_rate;
	
	p = acpid_get_p(atti_ctrl.pid_rate_roll,rate_error);
	// get i term
	i = acpid_get_integrator(atti_ctrl.pid_rate_roll);
	// update i term as long as we haven't breached the limits or the I term will certainly reduce
	// //没有达到限制值直接积分,达到了限制值则如果积分减小才进行
	if(!apmotor.limit.roll_pitch || ((i>0&&rate_error<0)||(i<0&&rate_error>0)))
	{
			i = acpid_get_i(atti_ctrl.pid_rate_roll,rate_error,atti_ctrl.dt);
	}
	
	 // get d term
	d = acpid_get_d(atti_ctrl.pid_rate_roll,rate_error,atti_ctrl.dt);
	
	return constrain_float((p+i+d), -AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX, AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX);

}



float rate_bf_to_motor_pitch(float rate_target_cds)
{
	float p,i,d;            // used to capture pid values for logging
	float current_rate;     // this iteration's rate
	float rate_error;       // simply target_rate - current_rate
	
	// get current rate
  // To-Do: make getting gyro rates more efficient?
	current_rate = (ahrs.omega.y * AC_ATTITUDE_CONTROL_DEGX100);		//期望速率
	
	// calculate error and call pid controller
	rate_error = rate_target_cds - current_rate;												//误差
	p = acpid_get_p(atti_ctrl.pid_rate_pitch,rate_error);							//比例
	// get i term
  i = acpid_get_integrator(atti_ctrl.pid_rate_pitch);								//积分
	// update i term as long as we haven't breached the limits or the I term will certainly reduce
	// //没有达到限制值直接积分,达到了限制值则如果积分减小才进行
	if(!apmotor.limit.roll_pitch || ((i>0&&rate_error<0)||(i<0&&rate_error>0)))
	{
			i = acpid_get_i(atti_ctrl.pid_rate_pitch,rate_error,atti_ctrl.dt);
	}
	
	 // get d term
  d = acpid_get_d(atti_ctrl.pid_rate_pitch,rate_error,atti_ctrl.dt);//微分
	
	return constrain_float((p+i+d), -AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX, AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX);

}

float rate_bf_to_motor_yaw(float rate_target_cds)
{
	float p,i,d;            // used to capture pid values for logging
	float current_rate;     // this iteration's rate
	float rate_error;       // simply target_rate - current_rate
	
	// get current rate
  // To-Do: make getting gyro rates more efficient?
	current_rate = (ahrs.omega.z * AC_ATTITUDE_CONTROL_DEGX100);		//期望速率
	
	// calculate error and call pid controller
	rate_error = rate_target_cds - current_rate;												//误差
	p = acpid_get_p(atti_ctrl.pid_rate_yaw,rate_error);							//比例
	// get i term
  i = acpid_get_integrator(atti_ctrl.pid_rate_yaw);								//积分
	// update i term as long as we haven't breached the limits or the I term will certainly reduce
	// //没有达到限制值直接积分,达到了限制值则如果积分减小才进行
	if(!apmotor.limit.yaw || ((i>0&&rate_error<0)||(i<0&&rate_error>0)))
	{
			i = acpid_get_i(atti_ctrl.pid_rate_yaw,rate_error,atti_ctrl.dt);
	}
	
	 // get d term
  d = acpid_get_d(atti_ctrl.pid_rate_yaw,rate_error,atti_ctrl.dt);//微分
	
	return constrain_float((p+i+d), -AC_ATTITUDE_RATE_YAW_CONTROLLER_OUT_MAX, AC_ATTITUDE_RATE_YAW_CONTROLLER_OUT_MAX);
}

void relax_bf_rate_controller()
{
	vector3f_t gyro;
	vector3f_scale(&ahrs.omega,AC_ATTITUDE_CONTROL_DEGX100,&gyro);
	vector3f_copy(&gyro,&atti_ctrl.rate_bf_target);
}

void set_yaw_target_to_current_heading()
{
	atti_ctrl.angle_ef_target.z = ahrs.yaw_sensor;
}

void set_throttle_out(int16_t throttle_out,uint8_t apply_angle_boost)
{
	if(apply_angle_boost)
	{
		apmotor.rc_throttle->servo_out = apmotor_get_angle_boost(throttle_out);
	}
	else
	{
		apmotor.rc_throttle->servo_out = throttle_out;
		apmotor.angle_boost = 0;
	}
}


void rate_controller_run()
{
	apmotor.rc_roll->servo_out =  rate_bf_to_motor_roll(atti_ctrl.rate_bf_target.x);		//motor_set_roll
	apmotor.rc_pitch->servo_out =  rate_bf_to_motor_pitch(atti_ctrl.rate_bf_target.y);	
	apmotor.rc_yaw->servo_out =  rate_bf_to_motor_yaw(atti_ctrl.rate_bf_target.z);
}



void update_ef_yaw_angle_and_error(float yaw_rate_ef, vector3f_t *angle_ef_error, float overshoot_max)
{
	// calculate angle error with maximum of +- max angle overshoot
	
	angle_ef_error->z = wrap_180_cd(atti_ctrl.angle_ef_target.z - ahrs.yaw_sensor);		  //本来就有的误差??
	angle_ef_error->z = constrain_float(angle_ef_error->z, -overshoot_max, overshoot_max);//限幅
	// update yaw angle target to be within max angle overshoot of our current heading
	atti_ctrl.angle_ef_target.z = angle_ef_error->z + ahrs.yaw_sensor;					//重新设置期望角度

	// increment the yaw angle target
	atti_ctrl.angle_ef_target.z += yaw_rate_ef * atti_ctrl.dt;							//期望速率积分得新期望角度
	atti_ctrl.angle_ef_target.z = wrap_360_cd(atti_ctrl.angle_ef_target.z);
}

// update_ef_roll_angle_and_error - update _angle_ef_target.x using an earth frame roll rate request
void update_ef_roll_angle_and_error(float roll_rate_ef, vector3f_t *angle_ef_error, float overshoot_max)
{
	// calculate angle error with maximum of +- max angle overshoot
	angle_ef_error->x = wrap_180_cd(atti_ctrl.angle_ef_target.x - ahrs.roll_sensor);		//角度误差,上次的期望角度?
	angle_ef_error->x = constrain_float(angle_ef_error->x, -overshoot_max, overshoot_max);	//误差限幅

	// update roll angle target to be within max angle overshoot of our roll angle
	atti_ctrl.angle_ef_target.x = angle_ef_error->x + ahrs.roll_sensor;						//更新期望角度,误差在一定范围内
	// increment the roll angle target新的期望角度,积分速度根据误差设置(曲线拟合得到)
	atti_ctrl.angle_ef_target.x += roll_rate_ef * atti_ctrl.dt;						//期望角速度积分得到期望角度,下次用？？
	atti_ctrl.angle_ef_target.x = wrap_180_cd(atti_ctrl.angle_ef_target.x);
}

void update_ef_pitch_angle_and_error(float pitch_rate_ef, vector3f_t *angle_ef_error, float overshoot_max)
{
	// calculate angle error with maximum of +- max angle overshoot
	// To-Do: should we do something better as we cross 90 degrees?
	angle_ef_error->y = wrap_180_cd(atti_ctrl.angle_ef_target.y - ahrs.pitch_sensor);
	angle_ef_error->y = constrain_float(angle_ef_error->y, -overshoot_max, overshoot_max);

	// update pitch angle target to be within max angle overshoot of our pitch angle
	atti_ctrl.angle_ef_target.y = angle_ef_error->y + ahrs.pitch_sensor;

	// increment the pitch angle target
	atti_ctrl.angle_ef_target.y += pitch_rate_ef *atti_ctrl.dt;
	atti_ctrl.angle_ef_target.y = wrap_180_cd(atti_ctrl.angle_ef_target.y);
}


void frame_conversion_ef_to_bf(const vector3f_t *ef_vector, vector3f_t *bf_vector)
{
	// convert earth frame rates to body frame rates	，欧拉角变化率转角速度
	bf_vector->x = ef_vector->x - ahrs.sin_pitch * ef_vector->z;		
	bf_vector->y = ahrs.cos_roll  * ef_vector->y + ahrs.sin_roll * ahrs.cos_pitch * ef_vector->z;
	bf_vector->z = -ahrs.sin_roll * ef_vector->y + ahrs.cos_pitch * ahrs.cos_roll * ef_vector->z;
}
//uint8_t frame_conversion_bf_to_ef(const vector3f_t *bf_vector, vector3f_t *ef_vector)
//{
//	// avoid divide by zero
//	if (ahrs.cos_pitch == 0.0f) {
//		return 0;
//	}
//	// convert earth frame angle or rates to body frame
//	ef_vector->x = bf_vector->x + ahrs.sin_roll * (ahrs.sin_pitch / ahrs.cos_pitch) * bf_vector->y + ahrs.cos_roll * (ahrs.sin_pitch / ahrs.cos_pitch) * bf_vector->z;
//	ef_vector->y = ahrs.cos_roll  * bf_vector->y - ahrs.sin_roll * bf_vector->z;
//	ef_vector->z = (ahrs.sin_roll / ahrs.cos_pitch) * bf_vector->y + (_ahrs.cos_roll / _ahrs.cos_pitch) * bf_vector->z;
//	return 1;
//}


void update_rate_bf_targets()
{
	// stab roll calculation
	atti_ctrl.rate_bf_target.x = atti_ctrl.p_angle_roll->kp * atti_ctrl.angle_bf_error.x;
	// constrain roll rate request
	if (atti_ctrl.flags.limit_angle_to_rate_request)		//期望速率限幅
	{//默认true ,默认180度/s
		atti_ctrl.rate_bf_target.x = constrain_float(atti_ctrl.rate_bf_target.x, -atti_ctrl.angle_rate_rp_max, atti_ctrl.angle_rate_rp_max);
	}

	// stab pitch calculation
	atti_ctrl.rate_bf_target.y = atti_ctrl.p_angle_pitch->kp * atti_ctrl.angle_bf_error.y;
	// constrain pitch rate request
	if (atti_ctrl.flags.limit_angle_to_rate_request) 
	{//默认true ,默认18000
		atti_ctrl.rate_bf_target.y = constrain_float(atti_ctrl.rate_bf_target.y, -atti_ctrl.angle_rate_rp_max, atti_ctrl.angle_rate_rp_max);
	}

	// stab yaw calculation
	atti_ctrl.rate_bf_target.z = atti_ctrl.p_angle_yaw->kp * atti_ctrl.angle_bf_error.z;
	// constrain yaw rate request
	if (atti_ctrl.flags.limit_angle_to_rate_request)
	{//默认true ,默认9000
		atti_ctrl.rate_bf_target.z = constrain_float(atti_ctrl.rate_bf_target.z, -atti_ctrl.angle_rate_y_max, atti_ctrl.angle_rate_y_max);
	}

	atti_ctrl.rate_bf_target.x += atti_ctrl.angle_bf_error.y * ahrs.omega.z;	//???what???
	atti_ctrl.rate_bf_target.y += -atti_ctrl.angle_bf_error.x * ahrs.omega.z;	//wtf?????
}



void angle_ef_roll_pitch_rate_ef_yaw_smooth(float roll_angle_ef, float pitch_angle_ef, float yaw_rate_ef, float smoothing_gain)
{
	vector3f_t angle_ef_error;		// earth frame angle errors,角度误差

	float rate_ef_desired;			//世界坐标系到期望速率
	float angle_to_target;			//到期望的角度
	float rate_change_limit;		//速度改变限幅

	if (atti_ctrl.accel_rp_max > 0.0f) 
	{//有最大旋转加速度限制
		// calculate earth-frame feed forward roll rate using linear response when close to the target, sqrt response when we're further away
		
		float linear_angle;
		// sanity check smoothing gain
		smoothing_gain = constrain_float(smoothing_gain, 1.0f, 50.0f);					//平滑增益限幅 1-50,也大响应越灵敏????
		linear_angle = atti_ctrl.accel_rp_max / (smoothing_gain*smoothing_gain);//旋转加速度限制,gain越大,速度限制越小??
		rate_change_limit = atti_ctrl.accel_rp_max * atti_ctrl.dt;							//最大角速度该变量,0???

		///////////////////////////////////////////////////////////
		angle_to_target = roll_angle_ef - atti_ctrl.angle_ef_target.x;	//期望角度改变量
		//根据角度误差得到期望速度作为前馈？
		if (angle_to_target > linear_angle)			//????	//该变量过大??
		{						
			rate_ef_desired = safe_sqrt(2.0f*atti_ctrl.accel_rp_max*((float)fabs(angle_to_target) - (linear_angle / 2.0f)));
		}
		else if (angle_to_target < -linear_angle) 
		{
			rate_ef_desired = -safe_sqrt(2.0f*atti_ctrl.accel_rp_max*((float)fabs(angle_to_target) - (linear_angle / 2.0f)));
		}
		else 
		{
			rate_ef_desired = smoothing_gain*angle_to_target;
		}
		//期望速度该变量限幅
		atti_ctrl.rate_ef_desired.x = constrain_float(rate_ef_desired, atti_ctrl.rate_ef_desired.x - rate_change_limit, atti_ctrl.rate_ef_desired.x + rate_change_limit);
		// update earth-frame roll angle target using desired roll rate获取角度误差???，期望速度积分得误差
		update_ef_roll_angle_and_error(atti_ctrl.rate_ef_desired.x, &angle_ef_error, AC_ATTITUDE_RATE_STAB_ROLL_OVERSHOOT_ANGLE_MAX);
		
		
		///////////////////////////////////////////////////////////////
		// calculate earth-frame feed forward pitch rate using linear response when close to the target, sqrt response when we're further away
		angle_to_target = pitch_angle_ef - atti_ctrl.angle_ef_target.y;
		if (angle_to_target > linear_angle)
		{
			rate_ef_desired = safe_sqrt(2.0f*atti_ctrl.accel_rp_max*((float)fabs(angle_to_target) - (linear_angle / 2.0f)));
		}
		else if (angle_to_target < -linear_angle) {
			rate_ef_desired = -safe_sqrt(2.0f*atti_ctrl.accel_rp_max*((float)fabs(angle_to_target) - (linear_angle / 2.0f)));
		}
		else
		{
			rate_ef_desired = smoothing_gain*angle_to_target;
		}

		atti_ctrl.rate_ef_desired.y = constrain_float(rate_ef_desired, atti_ctrl.rate_ef_desired.y - rate_change_limit, atti_ctrl.rate_ef_desired.y + rate_change_limit);
		// update earth-frame pitch angle target using desired pitch rate 
		update_ef_pitch_angle_and_error(atti_ctrl.rate_ef_desired.y, &angle_ef_error, AC_ATTITUDE_RATE_STAB_PITCH_OVERSHOOT_ANGLE_MAX);
	
	}
	else //没有最大加速度限制，直接求误差，前馈置0
	{
		// target roll and pitch to desired input roll and pitch
		atti_ctrl.angle_ef_target.x = roll_angle_ef;																					//直接给期望角度,为何不先限幅??外部限幅了??
		angle_ef_error.x = wrap_180_cd_float(atti_ctrl.angle_ef_target.x - ahrs.roll_sensor); //roll角度误差

		atti_ctrl.angle_ef_target.y = pitch_angle_ef;
		angle_ef_error.y = wrap_180_cd_float(atti_ctrl.angle_ef_target.y - ahrs.pitch_sensor);

		// set roll and pitch feed forward to zero
		atti_ctrl.rate_ef_desired.x = 0;
		atti_ctrl.rate_ef_desired.y = 0;
	}
	
	// constrain earth-frame angle targets
	//期望角度限幅，去掉误差再限幅有什么用？？
	atti_ctrl.angle_ef_target.x = constrain_float(atti_ctrl.angle_ef_target.x, -g.angle_max, g.angle_max);
	atti_ctrl.angle_ef_target.y = constrain_float(atti_ctrl.angle_ef_target.y, -g.angle_max, g.angle_max);

	//yaw计算
	if (atti_ctrl.accel_y_max > 0.0f)
	{//有最大旋转加速度限制
		float rate_change;
		// set earth-frame feed forward rate for yaw
		rate_change_limit = atti_ctrl.accel_y_max * atti_ctrl.dt;//速度变化量限制
		rate_change = yaw_rate_ef - atti_ctrl.rate_ef_desired.z;//前馈？？？
		rate_change = constrain_float(rate_change, -rate_change_limit, rate_change_limit);

		atti_ctrl.rate_ef_desired.z += rate_change;
		// calculate yaw target angle and angle error
		//计算角度误差
		update_ef_yaw_angle_and_error(atti_ctrl.rate_ef_desired.z, &angle_ef_error, AC_ATTITUDE_RATE_STAB_YAW_OVERSHOOT_ANGLE_MAX);
	}
	else 
	{	//没有最大旋转加速度限制
		// set yaw feed forward to zero
		atti_ctrl.rate_ef_desired.z = yaw_rate_ef;		//期望方向速度?
		//用期望速率计算角度误差,限制最大误差，最大10度
		update_ef_yaw_angle_and_error(atti_ctrl.rate_ef_desired.z, &angle_ef_error, AC_ATTITUDE_RATE_STAB_YAW_OVERSHOOT_ANGLE_MAX);
	}

	// convert earth-frame angle errors to body-frame angle errors
	frame_conversion_ef_to_bf(&angle_ef_error, &atti_ctrl.angle_bf_error);	//角度误差转到body
	// convert body-frame angle errors to body-frame rate targets
	update_rate_bf_targets();//angle_bf_error->rate_bf_target,P控制					//简单P控制得到body的期望速率并限幅


	// add body frame rate feed forward,rate_ef_desired???
	if (atti_ctrl.rate_bf_ff_enabled) //前馈
	{
		// convert earth-frame feed forward rates to body-frame feed forward rates
		frame_conversion_ef_to_bf(&atti_ctrl.rate_ef_desired, &atti_ctrl.rate_bf_desired);
		vector3f_add(&atti_ctrl.rate_bf_target, &atti_ctrl.rate_bf_desired, NULL);
	}
	else //只进行yaw反馈？
	{	
		atti_ctrl.rate_ef_desired.x = 0;
		atti_ctrl.rate_ef_desired.y = 0;
		//convert earth-frame feed forward rates to body-frame feed forward rates
		frame_conversion_ef_to_bf(&atti_ctrl.rate_ef_desired, &atti_ctrl.rate_bf_desired);
		vector3f_add(&atti_ctrl.rate_bf_target, &atti_ctrl.rate_bf_desired, NULL);			//yaw需要进行前馈
	}

}


void attitude_control_set_dt(float delta_sec)
{
	atti_ctrl.dt = delta_sec;
		
	acpid_set_d_alpha(atti_ctrl.pid_rate_roll,AC_ATTITUDE_RATE_RP_PID_DTERM_FILTER,delta_sec);
	acpid_set_d_alpha(atti_ctrl.pid_rate_pitch,AC_ATTITUDE_RATE_RP_PID_DTERM_FILTER,delta_sec);
	acpid_set_d_alpha(atti_ctrl.pid_rate_yaw,AC_ATTITUDE_RATE_RP_PID_DTERM_FILTER,delta_sec);
	
}

void attitude_control_init()
{
	atti_ctrl.p_angle_roll = &g.p_stabilize_roll;
	atti_ctrl.p_angle_pitch = &g.p_stabilize_pitch;
	atti_ctrl.p_angle_yaw = &g.p_stabilize_yaw;

	atti_ctrl.pid_rate_roll = &g.pid_rate_roll;
	atti_ctrl.pid_rate_pitch = &g.pid_rate_pitch;
	atti_ctrl.pid_rate_yaw = &g.pid_rate_yaw;

	atti_ctrl.flags.limit_angle_to_rate_request = 1;						//1,期望速率限幅
	atti_ctrl.angle_rate_rp_max = AC_ATTITUDE_CONTROL_RATE_RP_MAX_DEFAULT;
	atti_ctrl.angle_rate_y_max = AC_ATTITUDE_CONTROL_RATE_Y_MAX_DEFAULT;	

	//用于前馈控制
	atti_ctrl.accel_rp_max = AC_ATTITUDE_CONTROL_ACCEL_RP_MAX_DEFAULT;
	atti_ctrl.accel_y_max = AC_ATTITUDE_CONTROL_ACCEL_Y_MAX_DEFAULT;
	atti_ctrl.rate_bf_ff_enabled = AC_ATTITUDE_CONTROL_RATE_BF_FF_DEFAULT;	//0,关闭前馈

	attitude_control_set_dt(MAIN_LOOP_SECONDS);			//10ms
	
}




















