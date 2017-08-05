#ifndef _AHRS_DCM_H_
#define _AHRS_DCM_H_
#include "sys.h"
#include "matrix3f.h"
#include "vector3f.h"



#define AP_AHRS_TRIM_LIMIT 10.0f        // maximum trim angle in degrees
#define AP_AHRS_RP_P_MIN   0.05f        // minimum value for AHRS_RP_P parameter
#define AP_AHRS_YAW_P_MIN  0.05f        // minimum value for AHRS_YAW_P parameter
#define SPIN_RATE_LIMIT 20

typedef struct 
{
	matrix3f_t dcm_matrix;							//传感器的姿态,传感器的数据转换在此矩阵惊醒
	matrix3f_t body_dcm_matrix;					//机体的姿态,欧拉角由此矩阵计算,用于控制器
	
	// flags structure
	struct ahrs_flags 
	{
		uint8_t have_initial_yaw        : 1;    // whether the yaw value has been intialised with a reference
		uint8_t fast_ground_gains       : 1;    // should we raise the gain on the accelerometers for faster convergence, used when disarmed for ArduCopter
		uint8_t fly_forward             : 1;    // 1 if we can assume the aircraft will be flying forward on its X axis
		uint8_t correct_centrifugal     : 1;    // 1 if we should correct for centrifugal forces (allows arducopter to turn this off when motors are disarmed)
		uint8_t wind_estimation         : 1;    // 1 if we should do wind estimation
		uint8_t armed                   : 1;    // 1 if we are armed for flight
	}flags;
		
	
	
	//欧拉角
	float roll;								//units:rad
	float pitch;
	float yaw;			
	
	int32_t roll_sensor;				//units:centi-degree
	int32_t pitch_sensor;
	int32_t yaw_sensor;
	//欧拉角三角函数值
	float cos_roll,cos_pitch,cos_yaw;
	float sin_roll,sin_pitch,sin_yaw;
	
	
	//角速度 from ins.gyro
	vector3f_t omega;						//rad
	vector3f_t omega_P;						//
	vector3f_t omega_yaw_P;
	vector3f_t omega_I;						//
	vector3f_t omega_I_sum;						//
	float omega_I_sum_time;

	
	vector3f_t accel_ef;				//加速度转到世界坐标系,m/s2
	
	//
	vector3f_t ra_sum;				//accel_ef积分,用于gyro 漂移矫正
	float ra_deltat;					//积分时间
	uint32_t ra_sum_start;			//???
	
	
	//
	float error_rp_sum;				//误差模长累加???
	uint16_t error_rp_count;
	
	float error_yaw_sum;				//???
	uint16_t error_yaw_count;
	float error_yaw_last;

	//renorm,?????
	float renorm_val_sum;
	uint16_t renorm_val_count;


	//
	float ki;							//
	float ki_yaw;					//
	
	// these are public for ArduCopter
	float kp_yaw;				//default = 0.1
	float kp;						//default = 0.1
	float gyro_drift_limit;
	
	// a vector to capture the difference between the controller and body frames
	vector3f_t trim;			//AP!!	
	
	uint32_t compass_last_update;
	float gps_gain; 	 //defalult = 1.0

	// last time AHRS failed in milliseconds,
	uint32_t last_failure_ms;		//dcm出现错误时间
}ahrs_dcm_t;
extern ahrs_dcm_t ahrs;

void ahrs_dcm_init(void);
void ahrs_dcm_update(void);
void ahrs_startup_ground(uint8_t force_gyro_cal);
#endif

