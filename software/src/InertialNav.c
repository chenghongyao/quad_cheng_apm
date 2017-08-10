#include "InertialNav.h"
#include "barometer.h"
#include "ahrs_dcm.h"
#include "cmath.h"
#include "vector2f.h"

void InertialNav_init(void)
{
	inav.time_constant_xy = AP_INTERTIALNAV_TC_XY;	//2.5
	
	inav.time_constant_z = AP_INTERTIALNAV_TC_Z;	//3.0

	inav.flags.baro_glitching = 0;
	
	vector3f_zero(&inav.position_base);
	vector3f_zero(&inav.position_correction);
	vector3f_zero(&inav.position);
	vector3f_zero(&inav.position_error);
	vector3f_zero(&inav.accel_correction_hbf);
	vector3f_zero(&inav.velocity);

	AP_BufferFloat_Init(&inav.hist_position_estimate_x, 5);
	AP_BufferFloat_Init(&inav.hist_position_estimate_y, 5);
	AP_BufferFloat_Init(&inav.hist_position_estimate_z, 5);

	InertialNav_udate_gains();
}

void InertialNav_udate_gains()
{
	// X & Y axis time constant
	if (inav.time_constant_xy == 0.0f)
	{			//不矫正??
		inav.k1_xy = inav.k2_xy = inav.k3_xy = 0.0f;
	}
	else
	{
		inav.k1_xy = 3.0f / inav.time_constant_xy;     // 1/a,1/a^2,1/a^3
		inav.k2_xy = 3.0f / (inav.time_constant_xy*inav.time_constant_xy);
		inav.k3_xy = 1.0f / (inav.time_constant_xy*inav.time_constant_xy*inav.time_constant_xy);
	}

	// Z axis time constant
	if (inav.time_constant_z == 0.0f) 
	{
		inav.k1_z = inav.k2_z = inav.k3_z = 0.0f;
	}
	else
	{
		inav.k1_z = 3.0f / inav.time_constant_z;
		inav.k2_z = 3.0f / (inav.time_constant_z*inav.time_constant_z);
		inav.k3_z = 1.0f / (inav.time_constant_z*inav.time_constant_z*inav.time_constant_z);
	}
}


void InertialNav_set_altitude(float new_altitude)
{
	inav.position_base.z = new_altitude;
	inav.position_correction.z = 0;
	inav.position.z = new_altitude; // _position = _position_base + _position_correction
	AP_BufferFloat_Clear(&inav.hist_position_estimate_z);
}

float hist_base = 0;
void InertialNav_correct_with_baro(float baro_alt,float dt)
{
	static uint8_t first_read = 0;
	if (dt > 0.5f)return;

	if (first_read <= 30)		//前10个数据,用于确定当前高度,position.z
	{
		InertialNav_set_altitude(baro_alt);
		first_read++;
	}

	if (barometer.flags.glitching)//气压计数据有问题
	{
		inav.position_error.z *= 0.89715f;
	}
	else
	{
		if (inav.flags.baro_glitching)		//上一个气压计数据有问题,
		{									//使用当前数据为实际高度
			InertialNav_set_altitude(baro_alt);
			inav.position_error.z = 0.0f;
		}
		else
		{
			//TODO:
			float hist_position_base_z;
			if (AP_BufferFloat_Full(&inav.hist_position_estimate_z)) 
			{	//so who add to the buffer满了
				hist_position_base_z = AP_BufferFloat_Front(&inav.hist_position_estimate_z);
			}
			else 
			{
				hist_position_base_z = inav.position_base.z;
			}

			// calculate error in position from baro with our estimate
			//位置误差
			inav.position_error.z = baro_alt - (hist_position_base_z + inav.position_correction.z);	
				
			hist_base = hist_position_base_z;
		}
	}
	inav.flags.baro_glitching = barometer.flags.glitching;
}


void InertialNav_check_baro()
{
	float dt;
	if (barometer.last_update != inav.baro_last_update)//气压计数据更新了	
	{
		dt = (float)(barometer.last_update - inav.baro_last_update)*0.001f;
		InertialNav_correct_with_baro(barometer.altitude, dt);//用气压计高度纠正position_error.z
		inav.baro_last_update = barometer.last_update;
	}
}


float hist_push = 0;
void InertialNav_update(float dt)
{
	vector3f_t accel_ef;
	vector2f_t position_error_hbf;
	vector2f_t accel_correction_ef;		
	vector3f_t velocity_increase;			
	float tmp;

	if (dt>0.1f)//100ms,数据延迟过长,丢弃
	{
		return;
	}
	if (inav.flags.ignore_error > 0)
	{
		inav.flags.ignore_error--;
	}

	InertialNav_check_baro();					//气压计高度 ->position_error.z
	vector3f_copy(&ahrs.accel_ef, &accel_ef);	
	accel_ef.z += GRAVITY_MSS;					//除去重力加速度
	vector3f_scale(&accel_ef, 100.0f, NULL);	//->单位转换，cm/s2
	accel_ef.z = -accel_ef.z;					//方向改为向上为正

// 	if (!_xy_enabled) 
//	{
// 		accel_ef.x = 0.0f;
// 		accel_ef.y = 0.0f;
// 	}

	//x,y的位置误差转换到body平面
	position_error_hbf.x = inav.position_error.x * ahrs.cos_yaw + inav.position_error.y * ahrs.sin_yaw;
	position_error_hbf.y = -inav.position_error.x * ahrs.sin_yaw + inav.position_error.y * ahrs.cos_yaw;

	//_k1_xy,_k1_z ....等在init中设置,

	//位置误差 ->	加速度修正量(body平面)
	tmp = inav.k3_xy * dt;
	inav.accel_correction_hbf.x += position_error_hbf.x * tmp;
	inav.accel_correction_hbf.y += position_error_hbf.y * tmp;
	inav.accel_correction_hbf.z += inav.position_error.z * inav.k3_z  * dt;
	
	//加速度修正后再转到earth ，convert horizontal body frame accel correction to earth frame,
	accel_correction_ef.x = inav.accel_correction_hbf.x * ahrs.cos_yaw - inav.accel_correction_hbf.y * ahrs.sin_yaw;
	accel_correction_ef.y = inav.accel_correction_hbf.x * ahrs.sin_yaw + inav.accel_correction_hbf.y * ahrs.cos_yaw;

	tmp = inav.k2_xy * dt;
	inav.velocity.x += inav.position_error.x * tmp;					//位置误差->速度修正
	inav.velocity.y += inav.position_error.y * tmp;
	inav.velocity.z += inav.position_error.z * inav.k2_z  * dt;

	tmp = inav.k1_xy * dt;
	inav.position_correction.x += inav.position_error.x * tmp;	//位置误差->位置修正量
	inav.position_correction.y += inav.position_error.y * tmp;
	inav.position_correction.z += inav.position_error.z * inav.k1_z * dt;

	// calculate velocity increase adding new acceleration from accelerometers
	//加速度积分得速度增量
	velocity_increase.x = (accel_ef.x + accel_correction_ef.x) * dt;
	velocity_increase.y = (accel_ef.y + accel_correction_ef.y) * dt;
	velocity_increase.z = (accel_ef.z + inav.accel_correction_hbf.z) * dt;

	// calculate new velocity
	// calculate new estimate of position
	//速度积分得基础位置
	inav.position_base.x += (inav.velocity.x + velocity_increase.x*0.5) * dt;//加速度二次积分得位置
	inav.position_base.y += (inav.velocity.y + velocity_increase.y*0.5) * dt;
	inav.position_base.z += (inav.velocity.z + velocity_increase.z*0.5) * dt;
	
	// update the corrected position estimate,基础位置加上位置修正量
	//position = base + correction;
	vector3f_add(&inav.position_base, &inav.position_correction, &inav.position);
	//velocity += velocity_increase;
	vector3f_add(&inav.velocity, &velocity_increase, NULL);


	// store 3rd order estimate (i.e. estimated vertical position) for future use
	AP_BufferFloat_Push_Back(&inav.hist_position_estimate_z, inav.position_base.z);
	hist_push =  inav.position_base.z;
	// store 3rd order estimate (i.e. horizontal position) for future use at 10hz
	inav.historic_xy_counter++;
	if (inav.historic_xy_counter >= AP_INTERTIALNAV_SAVE_POS_AFTER_ITERATIONS)
	{
		inav.historic_xy_counter = 0;
		AP_BufferFloat_Push_Back(&inav.hist_position_estimate_x, inav.position_base.x);
		AP_BufferFloat_Push_Back(&inav.hist_position_estimate_y, inav.position_base.y);
	}
}

