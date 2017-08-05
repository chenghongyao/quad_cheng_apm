#include "ahrs_dcm.h"
#include "matrix3f.h"
#include "vector3f.h"
#include "vector2f.h"
#include "cmath.h"
#include "string.h"
#include "AP_InertialSensor.h"
#include "stdint.h"
#include "Compass.h"

#include "AP_State.h"


//从角速度更新dcm
void ahrs_dcm_matrix_update(float dt)
{
	vector3f_t omega_relative;
	vector3f_copy(&ins.gyro,&ahrs.omega);		//读取角速度
	
	//加漂移量??omega+=omega_I,
	vector3f_add(&ahrs.omega,&ahrs.omega_I,NULL);	
	//(omega + omega_P + omega_yaw_P) * _G_Dt
	
	vector3f_add(&ahrs.omega,&ahrs.omega_P,&omega_relative);
	vector3f_add(&omega_relative,&ahrs.omega_yaw_P,NULL);
	vector3f_scale(&omega_relative,dt,NULL);
	//矩阵更新
	matrix3f_rotate(&ahrs.dcm_matrix,&omega_relative);
}

uint8_t ahrs_dcm_renorm(vector3f_t *a,vector3f_t *result)
{
	float renorm_val;
	
	renorm_val =  1.0f / vector3f_length(a);	//应该约为1
	
	ahrs.renorm_val_sum += renorm_val;		//????
	ahrs.renorm_val_count++;
	
	if (!(renorm_val < 2.0f && renorm_val > 0.5f))
	{//过大或过小
		// this is larger than it should get - log it as a warning
		if (!(renorm_val < 1.0e6f && renorm_val > 1.0e-6f))
		{
			// we are getting values which are way out of
			// range, we will reset the matrix and hope we
			// can recover our attitude using drift
			// correction before we hit the ground!
			return 0;
		}
	}
	//result = a * renorm_val;
	vector3f_scale(a,renorm_val,result);
	return 1;
}

void ahrs_dcm_reset(uint8_t recover_eulers)
{
	vector3f_zero(&ahrs.omega_I);
	vector3f_zero(&ahrs.omega_P);
	vector3f_zero(&ahrs.omega_yaw_P);
	vector3f_zero(&ahrs.omega);	//？？？
	
	if (recover_eulers && !isnan(ahrs.roll) && !isnan(ahrs.pitch) && !isnan(ahrs.yaw)) 
	{
		matrix3f_from_euler(&ahrs.dcm_matrix, ahrs.roll, ahrs.pitch, ahrs.yaw);
	} 
	else 
	{
		// otherwise make it flat
		matrix3f_from_euler(&ahrs.dcm_matrix,0,0,0);
	}
}

void ahrs_dcm_normalize(void)
{
	float error;
	vector3f_t t0,t1,t2;
	
	//第一行向量与第二行向量正交,内积结果应为0
	error = vector3f_dot(&ahrs.dcm_matrix.a,&ahrs.dcm_matrix.b);
	//t0 = a - 0.5*error*b
	vector3f_scale(&ahrs.dcm_matrix.b,-0.5f*error,&t0);
	vector3f_add(&t0,&ahrs.dcm_matrix.a,NULL);
	//t1 = b - 0.5*error*a
	vector3f_scale(&ahrs.dcm_matrix.a,-0.5f*error,&t1);
	vector3f_add(&t1,&ahrs.dcm_matrix.b,NULL);
	//c = a x b
	vector3f_cross(&t0,&t1,&t2);	
	
	 if (!ahrs_dcm_renorm(&t0, &ahrs.dcm_matrix.a) ||			//各行归一化
        !ahrs_dcm_renorm(&t1, &ahrs.dcm_matrix.b) ||
        !ahrs_dcm_renorm(&t2, &ahrs.dcm_matrix.c)) 
	 {			//无法归一化则使用上一次的结果,并复位修正值?
        // Our solution is blowing up and we will force back
        // to last euler angles
        ahrs.last_failure_ms = millis();
        ahrs_dcm_reset(1);
    }
}

void ahrs_dcm_check_matrix(void)
{
	if(matrix3f_isnan(&ahrs.dcm_matrix))
	{
		ahrs_dcm_reset(1);
		return;
	}
	
	if (!(ahrs.dcm_matrix.c.x < 1.0f &&//余弦值超过1
          ahrs.dcm_matrix.c.x > -1.0f))
	{
        // We have an invalid matrix. Force a normalisation.
			ahrs_dcm_normalize();		//正规化

			if (matrix3f_isnan(&ahrs.dcm_matrix)||fabsf(ahrs.dcm_matrix.c.x) > 10) 
			{//没有解决问题，只能复位了
					// normalisation didn't fix the problem! We're
					// in real trouble. All we can do is reset
				 ahrs_dcm_reset(1);
			}
    }
}

//限幅1-10,spin_rate越大,增益越大
float ahrs_dcm_Pgain(float spin_rate)
{
    if (spin_rate < radians(50)) {
        return 1.0f;
    }

    if (spin_rate > radians(500)) {
        return 10.0f;
    }
    return spin_rate/radians(50);
}



void ahrs_dcm_reset_gyro_drift(void)
{
  vector3f_zero(&ahrs.omega_I_sum);
	ahrs.omega_I_sum_time = 0.0f;
	vector3f_zero(&ahrs.omega_I);	
}

//水平面上的加速度的越小，权值越大
//减少由于roll和pitch导致的罗盘误差
float ahrs_dcm_yaw_gain(void)
{
    float VdotEFmag = pythagorous2(ahrs.accel_ef.x,ahrs.accel_ef.y);
    if (VdotEFmag <= 4.0f) 
		{
        return 0.2f*(4.5f - VdotEFmag);
    }
    return 0.1f;
}


float yaw_error_compass(void)
{
	vector3f_t *mag = &compass.field;
	vector2f_t rb;

	matrix3f_mulXY(&ahrs.dcm_matrix, mag, &rb);		//磁力向量转到世界坐标系,取X,Y分量
	vector2f_normalize(&rb);						//归一化
	if (vector2f_isinf(&rb))//无效
	{
		return 0.0;
	}
	return 0.0;
}
//XXXXX
void drift_correction_yaw()
{
	 uint8_t new_value = 0;
	float yaw_error;
	float yaw_deltat;
	float error_z;
	float spin_rate;


	if (compass.last_update_time != ahrs.compass_last_update)//罗盘数据更新了
	{
		yaw_deltat = (compass.last_update_time - ahrs.compass_last_update)*1.0e-6f;
		ahrs.compass_last_update = compass.last_update_time;
		//TODO:Compass_read
		if (!ahrs.flags.have_initial_yaw && Compass_read())		//航向未更新
		{
			float heading = Compass_calculate_heading(&ahrs.dcm_matrix);		//计算航向
			matrix3f_from_euler(&ahrs.dcm_matrix, ahrs.roll, ahrs.pitch, heading);	//初始化
			vector3f_zero(&ahrs.omega_yaw_P);
			ahrs.flags.have_initial_yaw = 1;
		}
		new_value = 1;
		yaw_error = yaw_error_compass();		//yaw误差
	}

	if (!new_value)//没有数据,减小修正值
	{
		vector3f_scale(&ahrs.omega_yaw_P, 0.97, NULL);
		return;
	}
	error_z = ahrs.dcm_matrix.c.z * yaw_error;					//误差转到body
	spin_rate = vector3f_length(&ahrs.omega);				
	if (ahrs.kp_yaw < AP_AHRS_YAW_P_MIN) 						//增益限幅
		ahrs.kp_yaw = AP_AHRS_YAW_P_MIN;
	ahrs.omega_yaw_P.z = error_z*ahrs_dcm_Pgain(spin_rate)*ahrs.kp_yaw*ahrs_dcm_yaw_gain();	//修正项
	if(ahrs.flags.fast_ground_gains)	//快速修正
	{
		ahrs.omega_yaw_P.z *= 8;
	}
	//
	if(yaw_deltat < 2.0f && spin_rate < radians(SPIN_RATE_LIMIT)) 		//如果失去参考超过2s，不要添加该项
	{
		// also add to the I term
		ahrs.omega_I_sum.z += error_z * ahrs.ki_yaw * yaw_deltat;
	}
	ahrs.error_yaw_sum += fabsf(yaw_error);
	ahrs.error_yaw_count ++;	
	
}

//!!!!accel方向!!!
//omega_I
//omega_P
//omega_yaw_P
void ahrs_dcm_drift_correction(float dt)
{
	vector3f_t vec_temp;

	vector3f_t GA_e = {0,0,-1.0f};		//世界坐标系的加速度(向上为正??),单位g
	vector3f_t GA_b = { 0.0f, 0.0f, 0.0f };
	vector3f_t error;
//	float error_length;
	float ra_scale;
	float spin_rate;

	
	//drift_correction_yaw();
	matrix3f_mul(&ahrs.dcm_matrix,&ins.accel,&ahrs.accel_ef);//加速度转到世界坐标系

	//世界坐标系的加速度积分200ms   ra_sum += accel_ef*dt;
	vector3f_scale(&ahrs.accel_ef,dt,&vec_temp);			//
	vector3f_add(&ahrs.ra_sum,&vec_temp,NULL);
	ahrs.ra_deltat += dt;		


	//200ms矫正一次？
	if(ahrs.ra_deltat < 0.2f)		//积分200ms
	{
		return;
	}

//	if(ahrs.ra_sum_start == 0)			//第一次进入
//	{
//		ahrs.ra_sum_start = millis();
//		return;
//	}
	
	//200ms,求平均，模化为1(g)
	ra_scale = 1.0f/(ahrs.ra_deltat*GRAVITY_MSS);
	vector3f_scale(&ahrs.ra_sum,ra_scale,NULL);		//对积分值求平均,归一化	得到的应该是向上为正
	vector3f_copy(&ahrs.ra_sum,&GA_b);						//没有GPS，直接作为GA_b
	vector3f_normalize(&GA_b);									//正规化,大小变为1	
	
	
	if(vector3f_isnan(&GA_b) || vector3f_isinf(&GA_b))
	{
		ahrs.last_failure_ms = millis();
		return;
	}

	vector3f_cross(&GA_b,&GA_e,&error);										//叉乘误差
//	error_length = vector3f_length(&error);								//误差模长
	matrix3f_mul_transpose(&ahrs.dcm_matrix,&error,NULL);	//误差转到机体上
	if(vector3f_isnan(&error) || vector3f_isinf(&error))	//出错
	{
		ahrs_dcm_check_matrix();
		ahrs.last_failure_ms = millis();
		return ;
	}
	
//	ahrs.error_rp_sum += error_length;			//误差模长累加
//	ahrs.error_rp_count ++;
	
	spin_rate = vector3f_length(&ahrs.omega);//角速度大小，作为修正权值
	if (ahrs.kp < AP_AHRS_RP_P_MIN) ahrs.kp = AP_AHRS_RP_P_MIN;//default = 0.2
	//根据误差获取omega_P,kp默认0.1,角速度越大,修正力度越大
	//角速度大小限幅50-500 -> 增益1-10
	vector3f_scale(&error,ahrs_dcm_Pgain(spin_rate)*ahrs.kp,&ahrs.omega_P);//误差修正值
	if(ahrs.flags.fast_ground_gains)	//快速修正
	{
		vector3f_scale(&ahrs.omega_P,8,NULL);
	}

	
	// accumulate some integrator error
	//旋转速度很小时对误差积分   omega_I +=  ki*error*dt 
	if (spin_rate < radians(SPIN_RATE_LIMIT)) //小于20度/s时进行积分
	{	
		vector3f_scale(&error,ahrs.ra_deltat*ahrs.ki,&vec_temp);		
		vector3f_add(&ahrs.omega_I_sum,&vec_temp,NULL);
		ahrs.omega_I_sum_time += ahrs.ra_deltat;	//积分时间
	}
	
	if(ahrs.omega_I_sum_time > 5)	//积分5s修正一次漂移
	{
		float change_limit = ahrs.gyro_drift_limit * ahrs.omega_I_sum_time;		//积分限制
		ahrs.omega_I_sum.x = constrain_float(ahrs.omega_I_sum.x, -change_limit, change_limit);
		ahrs.omega_I_sum.y = constrain_float(ahrs.omega_I_sum.y, -change_limit, change_limit);
		ahrs.omega_I_sum.z = constrain_float(ahrs.omega_I_sum.z, -change_limit, change_limit);
		vector3f_add(&ahrs.omega_I,&ahrs.omega_I_sum,NULL);
		vector3f_zero(&ahrs.omega_I_sum);		//清零
		ahrs.omega_I_sum_time = 0;
	}
	
	vector3f_zero(&ahrs.ra_sum);		//加速度积分清零，200ms，
	ahrs.ra_deltat = 0;							
	ahrs.ra_sum_start = millis();	
}



void ahrs_dcm_update_cd_values(void)
{
	ahrs.roll_sensor  = degrees(ahrs.roll) * 100;
    ahrs.pitch_sensor = degrees(ahrs.pitch) * 100;
    ahrs.yaw_sensor   = degrees(ahrs.yaw) * 100;
//    if (ahrs.yaw_sensor < 0)		
//        ahrs.yaw_sensor += 36000;
}

void ahrs_dcm_euler_angles(void)
{
	// body_dcm_matrix = dcm_matrix;
	matrix3f_copy(&ahrs.dcm_matrix,&ahrs.body_dcm_matrix);
	//对body_dcm进行矫正
	matrix3f_rotateXYinv(&ahrs.body_dcm_matrix,&ahrs.trim);//坐标系与控制器的误差，在水平面上，正值->坐标系正旋转
	matrix3f_to_euler(&ahrs.body_dcm_matrix,&ahrs.roll,&ahrs.pitch,&ahrs.yaw);
	
	ahrs_dcm_update_cd_values();
}

//获取欧拉角的三角函数
void ahrs_dcm_update_trig(void)
{
	vector2f_t yaw_vector;
	// sin_yaw, cos_yaw
	yaw_vector.x = ahrs.body_dcm_matrix.a.x;		//cθcψ
	yaw_vector.y = ahrs.body_dcm_matrix.b.x;		//cθsψ
	vector2f_normalize(&yaw_vector);
	ahrs.sin_yaw = constrain_float(yaw_vector.y, -1.0, 1.0);
	ahrs.cos_yaw = constrain_float(yaw_vector.x, -1.0, 1.0);

	
	// cos_roll, cos_pitch
	ahrs.cos_pitch = safe_sqrt(1 - (ahrs.body_dcm_matrix.c.x * ahrs.body_dcm_matrix.c.x));
	ahrs.cos_roll = ahrs.body_dcm_matrix.c.z / ahrs.cos_pitch;
	ahrs.cos_pitch = constrain_float(ahrs.cos_pitch, 0, 1.0);
	ahrs.cos_roll = constrain_float(ahrs.cos_roll, -1.0, 1.0); // this relies on constrain_float() of infinity doing the right thing,which it does do in avr-libc

	// sin_roll, sin_pitch
	ahrs.sin_pitch = -ahrs.body_dcm_matrix.c.x;
	ahrs.sin_roll = ahrs.body_dcm_matrix.c.y / ahrs.cos_pitch;
}


void ahrs_dcm_update(void)
{
	  float delta_t;		//
    // tell the IMU to grab some data	XXXXXX
    if(!inertial_sensor_update())		//处理采样的gyro,accel,求平均？，更新ins.accel ins.gyro[]
				return;
		
    // ask the IMU how much time this sensor reading represents
    delta_t = ins.delta_time;		//		

    // if the update call took more than 0.2 seconds then discard it,
    // otherwise we may move too far. This happens when arming motors 
    // in ArduCopter
    if (delta_t > 0.2f) //200ms，时间太长，不处理
		{		
				vector3f_zero(&ahrs.ra_sum);			//gps用？？
        ahrs.ra_deltat = 0;
        return;
    }

    // Integrate the DCM matrix using gyro inputs
    ahrs_dcm_matrix_update(delta_t);		//dcm 算法,陀螺仪更新旋转矩阵

    // Normalize the DCM matrix
    ahrs_dcm_normalize();								//保证矩阵为正交

    // Perform drift correction
		ahrs_dcm_drift_correction(delta_t);	//漂移校正	XXXXXX

    // paranoid check for bad values in the DCM matrix
    ahrs_dcm_check_matrix(); 							 //矩阵检查

    // Calculate pitch, roll, yaw for stabilization and navigation
    ahrs_dcm_euler_angles();							//计算欧拉角 ,roll,pitch,yaw,xx_sensor

    // update trig values including _cos_roll, cos_pitch
    ahrs_dcm_update_trig();		//计算6个欧拉角三角函数值以方便后面计算*/

}

void ahrs_dcm_init()
{
	//euler
	ahrs.roll = 0.0f;
	ahrs.pitch = 0.0f;
	ahrs.yaw = 0.0f;

	ahrs.roll_sensor = 0;
	ahrs.pitch_sensor = 0;
	ahrs.yaw_sensor = 0;

	ahrs.cos_roll = 1.0f;
	ahrs.cos_pitch = 1.0f;
	ahrs.cos_yaw = 1.0f;

	ahrs.sin_roll = 0.0f;
	ahrs.sin_pitch = 0.0f;
	ahrs.sin_yaw = 0.0f;
	

	ahrs_dcm_reset_gyro_drift();

	//omeaga_P
	vector3f_zero(&ahrs.omega_P);
	vector3f_zero(&ahrs.omega_yaw_P);

	//renorm，？？？？
	ahrs.renorm_val_sum = 0.0f;
	ahrs.renorm_val_count = 0;

	//error rp，加速度
	ahrs.error_rp_sum = 0.0f;
	ahrs.error_rp_count = 0;

	//error yaw
	ahrs.error_yaw_sum = 0.0f;
	ahrs.error_yaw_count = 0;
	ahrs.error_yaw_last = 0.0f;

	//ra_sum accel_ef积分
	ahrs.ra_sum_start = 0;
	ahrs.ra_deltat = 0.0f;
	vector3f_zero(&ahrs.ra_sum);	

	//dcm
	matrix3f_identify(&ahrs.dcm_matrix);
	ahrs.ki = 0.0087f;					//误差积分
	ahrs.ki_yaw = 0.01f;
	
	ahrs.flags.fast_ground_gains  = 0;

	ahrs.compass_last_update = 0;
	ahrs.flags.have_initial_yaw = 0;
	
	//!!!!!!!!!
	//修正力度
	ahrs.kp = 0.2f;
	ahrs.kp_yaw = 0.2f;
	//传感器与控制平面误差
	vector3f_zero(&ahrs.trim);
	ahrs.gyro_drift_limit = radians(0.5f / 60); //改变两0.5度每秒
}
/////////////////////////////////////////////////////////////////
void ahrs_startup_ground(uint8_t force_gyro_cal)
{
	// reset ahrs gyro bias
	if (force_gyro_cal) 
	{
		ahrs_dcm_reset_gyro_drift();//omega漂移量设为0
	}

	// setup fast AHRS gains to get right attitud
	ahrs.flags.fast_ground_gains = 1;
	// set landed flag
	set_land_complete(1);				//在地面
	set_land_complete_maybe(1);
}

