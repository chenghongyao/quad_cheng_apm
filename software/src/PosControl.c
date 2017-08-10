#include "PosControl.h"
#include "delay.h"
#include "InertialNav.h"
#include "paramter.h"
#include "AP_Motor.h"
#include "delay.h"
#include "ahrs_dcm.h"
#include "AttitudeControl.h"
#include "cmath.h"
#include "LowPassFilter.h"
#include "config.h"



void pos_control_set_dt(float dt)
{
	pos_ctrl.dt = dt;
	acpid_set_d_alpha(pos_ctrl.pid_alt_accel,POSCONTROL_ACCEL_Z_DTERM_FILTER,dt);
	low_pass_filter_set_fc(&pos_ctrl.vel_error_filter, dt, POSCONTROL_VEL_ERROR_CUTOFF_FREQ);
	low_pass_filter_set_fc(&pos_ctrl.accel_error_filter,dt,POSCONTROL_ACCEL_ERROR_CUTOFF_FREQ);
}



void pos_control_init(void)
{
	pos_ctrl.p_alt_pos = &g.p_alt_pos;
	pos_ctrl.p_alt_rate = &g.p_alt_rate;
	pos_ctrl.pid_alt_accel = &g.pid_alt_accel;

	pos_ctrl.throttle_hover  = 
	pos_ctrl.last_update_z_ms = 0;

	pos_ctrl.alt_max = 0;			//无高度限制
	pos_ctrl.leash_up_z = POSCONTROL_LEASH_LENGTH_MIN;			//高于当前的最大误差
	pos_ctrl.leash_down_z = POSCONTROL_LEASH_LENGTH_MIN;			//低于当前的最大误差
	pos_ctrl.speed_up_cms	= POSCONTROL_SPEED_UP;			//最大爬升速率
	pos_ctrl.speed_down_cms	= POSCONTROL_SPEED_DOWN;		//最大下降速率
	pos_ctrl.accel_z_cms = POSCONTROL_ACCEL_Z;				//最大垂直加速度,0时取默认值50cm/s2??
	
	pos_ctrl.flags.recalc_leash_z = 1;
	pos_ctrl.flags.reset_rate_to_accel_z = 1;
	pos_ctrl.flags.reset_accel_to_throttle = 1;

	pos_ctrl.throttle_hover = POSCONTROL_THROTTLE_HOVER;


	pos_ctrl.p_alt_pos = &g.p_alt_pos;
	pos_ctrl.p_alt_rate = &g.p_alt_rate;
	pos_ctrl.pid_alt_accel = &g.pid_alt_accel;

	vector3f_zero(&pos_ctrl.pos_target);
	vector3f_zero(&pos_ctrl.pos_error);

	vector3f_zero(&pos_ctrl.vel_desired);
	vector3f_zero(&pos_ctrl.vel_target);
	vector3f_zero(&pos_ctrl.accel_feedforward);
	vector3f_zero(&pos_ctrl.vel_last);
	vector3f_zero(&pos_ctrl.vel_error);

	vector3f_zero(&pos_ctrl.accel_error);

	
	pos_control_set_dt(MAIN_LOOP_SECONDS);
}

void pos_control_init_takeoff(void)
{
	pos_ctrl.pos_target.z = inav.position.z + POSCONTROL_TAKEOFF_JUMP_CM;		//起飞跳升距离
	pos_ctrl.flags.freeze_ff_z = 1;																					//使用前馈?
	pos_ctrl.pid_alt_accel->integrator = apmotor.rc_throttle->servo_out - pos_ctrl.throttle_hover;//初始积分值?
}

void pos_control_set_alt_target_from_climb_rate(float climb_rate_cms, float dt, uint8_t force_descend)
{
	// adjust desired alt if motors have not hit their limits
	// To-Do: add check of _limit.pos_down?
	//motor输出没有碰到限制值是对期望速率积分,
	//下降时油门达到下限则停止积分
	//上升时如果高度达到最大值时禁止积分
	if ((climb_rate_cms < 0 && (!apmotor.limit.throttle_lower || force_descend)) || (climb_rate_cms>0 && !apmotor.limit.throttle_upper && !pos_ctrl.limit.pos_up))
	{
		pos_ctrl.pos_target.z += climb_rate_cms * dt;
	}

	// do not let target alt get above limit
	//有高度限制时对期望高度限幅
	if (pos_ctrl.alt_max > 0 && pos_ctrl.pos_target.z > pos_ctrl.alt_max) 
	{
		pos_ctrl.pos_target.z = pos_ctrl.alt_max;
		pos_ctrl.limit.pos_up = 1;
	}
	pos_ctrl.vel_desired.z = climb_rate_cms;
}


/// calc_leash_length - calculates the horizontal leash length given a maximum speed, acceleration and position kP gain
float pos_control_calc_leash_length(float speed_cms, float accel_cms, float kP) 
{
	float leash_length;
	// sanity check acceleration and avoid divide by zero
	if (accel_cms <= 0.0f) 
	{
		accel_cms = POSCONTROL_ACCELERATION_MIN;
	}

	// avoid divide by zero
	if (kP <= 0.0f) 
	{
		return POSCONTROL_LEASH_LENGTH_MIN;
	}

	// calculate leash length
	if (speed_cms <= accel_cms / kP)
	{
		// linear leash length based on speed close in
		leash_length = speed_cms / kP;
	}
	else
	{
		// leash length grows at sqrt of speed further out??????
		leash_length = (accel_cms / (2.0f*kP*kP)) + (speed_cms*speed_cms / (2.0f*accel_cms));
	}

	// ensure leash is at least 1m long
	if (leash_length < POSCONTROL_LEASH_LENGTH_MIN)
	{
		leash_length = POSCONTROL_LEASH_LENGTH_MIN;
	}
	return leash_length;
}


void pos_control_calc_leash_length_z()
{
	if (pos_ctrl.flags.recalc_leash_z)		//需要重新计算
	{
		pos_ctrl.leash_up_z = pos_control_calc_leash_length(pos_ctrl.speed_up_cms, pos_ctrl.accel_z_cms, pos_ctrl.p_alt_pos->kp);//???
		pos_ctrl.leash_down_z = pos_control_calc_leash_length(-pos_ctrl.speed_down_cms, pos_ctrl.accel_z_cms, pos_ctrl.p_alt_pos->kp);
		pos_ctrl.flags.recalc_leash_z = 0;
	}
}

// accel_to_throttle - alt hold's acceleration controller
// calculates a desired throttle which is sent directly to the motors
void pos_control_accel_to_throttle(float accel_target_z)
{
	float z_accel_meas;			  // actual acceleration
	int32_t p, i, d;              // used to capture pid values for logging

	// Calculate Earth Frame Z acceleration
	z_accel_meas = -(ahrs.accel_ef.z + GRAVITY_MSS) * 100.0f;//当前加速度,向上为正

	// reset target altitude if this controller has just been engaged
	//求加速度误差

	if (pos_ctrl.flags.reset_accel_to_throttle)			//
	{
		// Reset Filter
		pos_ctrl.accel_error.z = 0;
		low_pass_filter_reset(&pos_ctrl.accel_error_filter, 0);
		pos_ctrl.flags.reset_accel_to_throttle = 0;
	}
	else	//加速度误差限幅滤波
	{
		// calculate accel error and Filter with fc = 2 Hz
		pos_ctrl.accel_error.z = low_pass_filter_apply(&pos_ctrl.accel_error_filter, constrain_float(accel_target_z - z_accel_meas, -32000, 32000));
	}

	//加速度pid
	// separately calculate p, i, d values for logging
	p = acpid_get_p(pos_ctrl.pid_alt_accel, pos_ctrl.accel_error.z);
	// get i term
	i = acpid_get_integrator(pos_ctrl.pid_alt_accel);
	// update i term as long as we haven't breached the limits or the I term will certainly reduce
	// To-Do: should this be replaced with limits check from attitude_controller?
	if ((!apmotor.limit.throttle_lower && !apmotor.limit.throttle_upper) || (i > 0 && pos_ctrl.accel_error.z < 0) || (i < 0 && pos_ctrl.accel_error.z>0)) 
	{	
		i = acpid_get_i(pos_ctrl.pid_alt_accel, pos_ctrl.accel_error.z, pos_ctrl.dt);
	}
	// get d term
	acpid_get_d(pos_ctrl.pid_alt_accel, pos_ctrl.accel_error.z, pos_ctrl.dt);


	// To-Do: pull min/max throttle from motors
	// To-Do: we had a contraint here but it's now removed, is this ok?  with the motors library handle it ok?
	//输出
	set_throttle_out((int16_t)p + i + d + pos_ctrl.throttle_hover, 1);
	// to-do add back in PID logging?
}


void pos_control_rate_to_accel_z()
{
	vector3f_t *curr_vel = &inav.velocity;	//当前速度
	float p;                                // used to capture pid values for logging
	float desired_accel;                    // the target acceleration if the accel based throttle is enabled, otherwise the output to be sent to the motors

	// check speed limits
	// To-Do: check these speed limits here or in the pos->rate controller
	//期望垂直速度限幅
	pos_ctrl.limit.vel_up = 0;
	pos_ctrl.limit.vel_down = 0;
	if (pos_ctrl.vel_target.z < pos_ctrl.speed_down_cms) 
	{
		pos_ctrl.vel_target.z = pos_ctrl.speed_down_cms;
		pos_ctrl.limit.vel_down = 1;
	}
	if (pos_ctrl.vel_target.z > pos_ctrl.speed_up_cms)
	{
		pos_ctrl.vel_target.z = pos_ctrl.speed_up_cms;
		pos_ctrl.limit.vel_up = 1;
	}

	// reset last velocity target to current target
	//上个期望速率设置为当前,前馈量置0
	if (pos_ctrl.flags.reset_rate_to_accel_z) 
	{
		pos_ctrl.vel_last.z = pos_ctrl.vel_target.z;
	}

	// feed forward desired acceleration calculation
	if (pos_ctrl.dt > 0.0f)
	{
		if (!pos_ctrl.flags.freeze_ff_z) 
		{
			pos_ctrl.accel_feedforward.z = (pos_ctrl.vel_target.z - pos_ctrl.vel_last.z) / pos_ctrl.dt;//期望加速度?
		}
		else 
		{
			//stop the feed forward being calculated during a known discontinuity
			pos_ctrl.flags.freeze_ff_z = 0;
		}
	}
	else 
	{
		pos_ctrl.accel_feedforward.z = 0.0f;
	}

	// store this iteration's velocities for the next iteration
	pos_ctrl.vel_last.z = pos_ctrl.vel_target.z;


	// reset velocity error and filter if this controller has just been engaged
	//速度误差转期望加速度
	if (pos_ctrl.flags.reset_rate_to_accel_z) 
	{
		// Reset Filter
		pos_ctrl.vel_error.z = 0;
		low_pass_filter_reset(&pos_ctrl.vel_error_filter, 0);
		desired_accel = 0;
		pos_ctrl.flags.reset_rate_to_accel_z = 0;
	}
	else //速度误差限幅滤波
	{
		// calculate rate error and filter with cut off frequency of 2 Hz
		pos_ctrl.vel_error.z = low_pass_filter_apply(&pos_ctrl.vel_error_filter, constrain_float(pos_ctrl.vel_target.z - curr_vel->z, -32000, 32000));

	}

	// calculate p
	p = pos_ctrl.p_alt_rate->kp * pos_ctrl.vel_error.z;				//速度环p控制,得期望加速度
	// consolidate and constrain target acceleration
	desired_accel = pos_ctrl.accel_feedforward.z + p;				//加入前馈
	desired_accel = constrain_int32(desired_accel, -32000, 32000);//期望加速度限幅

	// set target for accel based throttle controller
	pos_control_accel_to_throttle(desired_accel);					//期望加速度转油门
}

void pos_control_pos_to_rate_z()
{
	float curr_alt = inav.position.z;			//当前高度
	float linear_distance;						// half the distance we swap between linear and sqrt and the distance we offset sqrt.

	// clear position limit flags
	pos_ctrl.limit.pos_up = 0;				//初始化标志位
	pos_ctrl.limit.pos_down = 0;

	// calculate altitude error
	pos_ctrl.pos_error.z = pos_ctrl.pos_target.z - curr_alt;//高度误差

	// do not let target altitude get too far from current altitude
	//高度误差限幅
	if (pos_ctrl.pos_error.z > pos_ctrl.leash_up_z)
	{
		pos_ctrl.pos_target.z = curr_alt + pos_ctrl.leash_up_z;
		pos_ctrl.pos_error.z = pos_ctrl.leash_up_z;
		pos_ctrl.limit.pos_up = 1;
	}
	if (pos_ctrl.pos_error.z < -pos_ctrl.leash_down_z) 
	{
		pos_ctrl.pos_target.z = curr_alt - pos_ctrl.leash_down_z;
		pos_ctrl.pos_error.z = -pos_ctrl.leash_down_z;
		pos_ctrl.limit.pos_down = 1;
	}
	// check kP to avoid division by zero
	//根据高度误差,分段计算计算期望速度
	//经过仿真,作用很小~~,除非误差非常大(8m??),
	//增大kp会增加修正作用
	if (pos_ctrl.p_alt_pos->kp != 0.0f) 
	{
		linear_distance = pos_ctrl.accel_z_cms / (2.0f*pos_ctrl.p_alt_pos->kp*pos_ctrl.p_alt_pos->kp);//????
		if (pos_ctrl.pos_error.z > 2 * linear_distance)
		{
			pos_ctrl.vel_target.z = safe_sqrt(2.0f*pos_ctrl.accel_z_cms*(pos_ctrl.pos_error.z - linear_distance));
		}
		else if (pos_ctrl.pos_error.z < -2.0f*linear_distance)
		{
			pos_ctrl.vel_target.z = -safe_sqrt(2.0f*pos_ctrl.accel_z_cms*(-pos_ctrl.pos_error.z - linear_distance));
		}
		else
		{
			pos_ctrl.vel_target.z = acpid_get_p(pos_ctrl.p_alt_pos,pos_ctrl.pos_error.z);
		}
	}
	else
	{
		pos_ctrl.vel_target.z = 0;
	}

	// call rate based throttle controller which will update accel based throttle controller targets
	//期望速率转期望加速度
	pos_control_rate_to_accel_z();
}



/// update_z_controller - fly to altitude in cm above home
void pos_control_update_z_controller()
{
	// check time since last cast
	uint32_t now = millis();
	//控制周期大于200ms,复位
	if (now - pos_ctrl.last_update_z_ms > POSCONTROL_ACTIVE_TIMEOUT_MS)
	{
		pos_ctrl.flags.reset_rate_to_accel_z = 1;
		pos_ctrl.flags.reset_accel_to_throttle = 1;
	}
	pos_ctrl.last_update_z_ms = now;

	// check if leash lengths need to be recalculated
	//高度误差限幅值计算
	pos_control_calc_leash_length_z();		//??pos_ctrl.leash_up_z,pos_ctrl.leash_down_z

	// call position controller
	pos_control_pos_to_rate_z();		//控制器
}

void pos_control_set_speed_z(float speed_down, float speed_up)
{
	// ensure speed_down is always negative
	speed_down = (float)-fabs(speed_down);

	if (((float)fabs(pos_ctrl.speed_down_cms - speed_down) > 1.0f) || ((float)fabs(pos_ctrl.speed_up_cms - speed_up) > 1.0f))
	{
		pos_ctrl.speed_down_cms = speed_down;
		pos_ctrl.speed_up_cms = speed_up;
		pos_ctrl.flags.recalc_leash_z = 1;
	}
}
void pos_control_set_accel_z(float accel_cmss)
{
	if ((float)fabs(pos_ctrl.accel_z_cms - accel_cmss) > 1.0f)
	{
		pos_ctrl.accel_z_cms = accel_cmss;
		pos_ctrl.flags.recalc_leash_z = 1;
	}
}

/// get_stopping_point_z - sets stopping_point.z to a reasonable stopping altitude in cm above home
void pos_control_get_stopping_point_z(vector3f_t *stopping_point)
{
	float curr_pos_z = inav.position.z;
	float curr_vel_z = inav.velocity.z;

	float linear_distance;  // half the distance we swap between linear and sqrt and the distance we offset sqrt
	float linear_velocity;  // the velocity we swap between linear and sqrt

	// if position controller is active add current velocity error to avoid sudden jump in acceleration
	curr_vel_z += pos_ctrl.vel_error.z;


	// calculate the velocity at which we switch from calculating the stopping point using a linear function to a sqrt function
	linear_velocity = pos_ctrl.accel_z_cms / pos_ctrl.p_alt_pos->kp;

	if ((float)fabs(curr_vel_z) < linear_velocity)
	{
		// if our current velocity is below the cross-over point we use a linear function
		stopping_point->z = curr_pos_z + curr_vel_z / pos_ctrl.p_alt_pos->kp;
	}
	else 
	{
		linear_distance = pos_ctrl.accel_z_cms / (2.0f*pos_ctrl.p_alt_pos->kp*pos_ctrl.p_alt_pos->kp);
		if (curr_vel_z > 0)
		{
			stopping_point->z = curr_pos_z + (linear_distance + curr_vel_z*curr_vel_z / (2.0f*pos_ctrl.accel_z_cms));
		}
		else 
		{
			stopping_point->z = curr_pos_z - (linear_distance + curr_vel_z*curr_vel_z / (2.0f*pos_ctrl.accel_z_cms));
		}
	}
	stopping_point->z = constrain_float(stopping_point->z, curr_pos_z - POSCONTROL_STOPPING_DIST_Z_MAX, curr_pos_z + POSCONTROL_STOPPING_DIST_Z_MAX);
}
/// set_target_to_stopping_point_z - returns reasonable stopping altitude in cm above home
void pos_control_set_target_to_stopping_point_z()
{
	// check if z leash needs to be recalculated
	pos_control_calc_leash_length_z();
	pos_control_get_stopping_point_z(&pos_ctrl.pos_target);		//切换之后的高度停止点（防止当时飞机正在上升或下降）
}
