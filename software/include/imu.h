#ifndef _IMU_H
#define _IMU_H

#include "sys.h"
#include "quat.h"


#define IMU_SENSOR_CALI_COUNT		300
#define IMU_MAG_CALI_COUNT			1500		//20s??	

#define Kp 0.3f  
#define Ki  0.0f
#define IMU_INTEGRAL_LIM  DEG2RAD(2.0f)

typedef	struct 
{
	float accel_raw[3];			//传感器 加速度,m/s^2
	float gyro_raw[3];			//传感器 陀螺仪,deg/s
	
	float accel_gain[3];
	float accel_offset[3];	//加速度零偏,m/s^2
	float gyro_offset[3];		//陀螺仪零偏,deg/s


	float accel_b[3];						//加速度(机体坐标系),m/s^2
	float accel_w[3];						//加速度(世界坐标系),m/s^2
	float accel_w_offset[3];		//加速度(世界坐标系)零偏,除重力,m/s^2
	float gyro[3];							//陀螺仪,(机体坐标系),rad/s
	float gyro_deg[3];					//陀螺仪,(机体坐标系),deg/s
	
	float accel_yaw[2];
	
	float mag_raw[3];						//传感器 磁力计,
	float mag_b[3];							//磁力计(机体坐标系),
	float mag_gain[6];
	float mag_offset[3];				//磁力计零偏;
	float mag_w[3];							//磁力计(世界坐标系),
	float yaw_mag;							//倾角补偿
	float yaw_mag_raw;
	float orientation[3];				//姿态欧拉角,rad
	float roll;									//姿态角，度
	float pitch;
	float yaw;
	
	quaternion q;								//四元数
	float dcm[3][3];						//变换矩阵(R:body->world)


	//保存以加速计算
	float sin_roll;
	float cos_roll;
	float sin_pitch;
	float cos_pitch;
	float sin_yaw;
	float cos_yaw;

	uint8_t ready;
	uint8_t mag_ready;
	uint8_t acc_w_ready;
}imu_t;



extern imu_t mImu;


void imu_init(void);
void imu_update(float dt);
void imu_updateMag(float dt);

void imu_prepareAngle(void);
void quaternion2dcm(quaternion *q, float R[3][3]);
void dcm2angle(float R[3][3], float angel[3]);
void body2world(float R[3][3], float body[3], float world[3]);

#endif


