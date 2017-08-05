#ifndef __AP_INERTIAL_SENSOR_H__
#define __AP_INERTIAL_SENSOR_H__

#include "vector3f.h"



typedef struct
{
	vector3f_t gyro;			//rad
	vector3f_t accel;			//m/s2
	
	uint16_t sample_count;			//一次处理所需的采样样本数,init=2
	uint16_t sample_max;				//实际采样最大值记录
	uint16_t sum_count;					
	vector3f_t accel_sum;
	vector3f_t gyro_sum;
	
	uint32_t sample_period_usec;
	uint32_t next_sample_usec;
	uint32_t last_sample_usec;
	float delta_time;


	enum Rotation board_orientation;
	uint8_t have_sample:1;
	uint8_t	gyro_cal_ok : 1;
	uint8_t calibrated : 1;
	

	///////////////传感器矫正值
	vector3f_t accel_scale;
	float gyro_scale;
	vector3f_t accel_offset;
	vector3f_t gyro_offset;
}inertial_sensor_t;

extern inertial_sensor_t ins;


void inertial_sensor_wait_for_sample(void);
uint8_t inertial_sensor_update(void);
void inertial_sensor_init(void);

#endif

