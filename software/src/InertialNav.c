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
	{			//������??
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

	if (first_read <= 30)		//ǰ10������,����ȷ����ǰ�߶�,position.z
	{
		InertialNav_set_altitude(baro_alt);
		first_read++;
	}

	if (barometer.flags.glitching)//��ѹ������������
	{
		inav.position_error.z *= 0.89715f;
	}
	else
	{
		if (inav.flags.baro_glitching)		//��һ����ѹ������������,
		{									//ʹ�õ�ǰ����Ϊʵ�ʸ߶�
			InertialNav_set_altitude(baro_alt);
			inav.position_error.z = 0.0f;
		}
		else
		{
			//TODO:
			float hist_position_base_z;
			if (AP_BufferFloat_Full(&inav.hist_position_estimate_z)) 
			{	//so who add to the buffer����
				hist_position_base_z = AP_BufferFloat_Front(&inav.hist_position_estimate_z);
			}
			else 
			{
				hist_position_base_z = inav.position_base.z;
			}

			// calculate error in position from baro with our estimate
			//λ�����
			inav.position_error.z = baro_alt - (hist_position_base_z + inav.position_correction.z);	
				
			hist_base = hist_position_base_z;
		}
	}
	inav.flags.baro_glitching = barometer.flags.glitching;
}


void InertialNav_check_baro()
{
	float dt;
	if (barometer.last_update != inav.baro_last_update)//��ѹ�����ݸ�����	
	{
		dt = (float)(barometer.last_update - inav.baro_last_update)*0.001f;
		InertialNav_correct_with_baro(barometer.altitude, dt);//����ѹ�Ƹ߶Ⱦ���position_error.z
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

	if (dt>0.1f)//100ms,�����ӳٹ���,����
	{
		return;
	}
	if (inav.flags.ignore_error > 0)
	{
		inav.flags.ignore_error--;
	}

	InertialNav_check_baro();					//��ѹ�Ƹ߶� ->position_error.z
	vector3f_copy(&ahrs.accel_ef, &accel_ef);	
	accel_ef.z += GRAVITY_MSS;					//��ȥ�������ٶ�
	vector3f_scale(&accel_ef, 100.0f, NULL);	//->��λת����cm/s2
	accel_ef.z = -accel_ef.z;					//�����Ϊ����Ϊ��

// 	if (!_xy_enabled) 
//	{
// 		accel_ef.x = 0.0f;
// 		accel_ef.y = 0.0f;
// 	}

	//x,y��λ�����ת����bodyƽ��
	position_error_hbf.x = inav.position_error.x * ahrs.cos_yaw + inav.position_error.y * ahrs.sin_yaw;
	position_error_hbf.y = -inav.position_error.x * ahrs.sin_yaw + inav.position_error.y * ahrs.cos_yaw;

	//_k1_xy,_k1_z ....����init������,

	//λ����� ->	���ٶ�������(bodyƽ��)
	tmp = inav.k3_xy * dt;
	inav.accel_correction_hbf.x += position_error_hbf.x * tmp;
	inav.accel_correction_hbf.y += position_error_hbf.y * tmp;
	inav.accel_correction_hbf.z += inav.position_error.z * inav.k3_z  * dt;
	
	//���ٶ���������ת��earth ��convert horizontal body frame accel correction to earth frame,
	accel_correction_ef.x = inav.accel_correction_hbf.x * ahrs.cos_yaw - inav.accel_correction_hbf.y * ahrs.sin_yaw;
	accel_correction_ef.y = inav.accel_correction_hbf.x * ahrs.sin_yaw + inav.accel_correction_hbf.y * ahrs.cos_yaw;

	tmp = inav.k2_xy * dt;
	inav.velocity.x += inav.position_error.x * tmp;					//λ�����->�ٶ�����
	inav.velocity.y += inav.position_error.y * tmp;
	inav.velocity.z += inav.position_error.z * inav.k2_z  * dt;

	tmp = inav.k1_xy * dt;
	inav.position_correction.x += inav.position_error.x * tmp;	//λ�����->λ��������
	inav.position_correction.y += inav.position_error.y * tmp;
	inav.position_correction.z += inav.position_error.z * inav.k1_z * dt;

	// calculate velocity increase adding new acceleration from accelerometers
	//���ٶȻ��ֵ��ٶ�����
	velocity_increase.x = (accel_ef.x + accel_correction_ef.x) * dt;
	velocity_increase.y = (accel_ef.y + accel_correction_ef.y) * dt;
	velocity_increase.z = (accel_ef.z + inav.accel_correction_hbf.z) * dt;

	// calculate new velocity
	// calculate new estimate of position
	//�ٶȻ��ֵû���λ��
	inav.position_base.x += (inav.velocity.x + velocity_increase.x*0.5) * dt;//���ٶȶ��λ��ֵ�λ��
	inav.position_base.y += (inav.velocity.y + velocity_increase.y*0.5) * dt;
	inav.position_base.z += (inav.velocity.z + velocity_increase.z*0.5) * dt;
	
	// update the corrected position estimate,����λ�ü���λ��������
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

