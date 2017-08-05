#ifndef _PARAMTER_H_
#define _PARAMTER_H_


#include "RC_Channel.h"
#include "vector3f.h"
#include "AC_PID.h"


typedef struct
{
	
	//RC
	rcchannel_t         rc_1;
	rcchannel_t         rc_2;
	rcchannel_t         rc_3;		//油门
	rcchannel_t         rc_4;
	rcchannel_t         rc_5;
	rcchannel_t         rc_6;
	rcchannel_t         rc_7;
	rcchannel_t         rc_8;


	int16_t 						angle_max;
	float 							acro_yaw_p;
	int8_t 							rc_feel_rp;
	int8_t 							arming_check;
	int8_t 							esc_calibrate;


	// Throttle
	int16_t        throttle_min;
	int16_t        throttle_max;
	int16_t        throttle_mid;
	int8_t         failsafe_throttle;
	int16_t        failsafe_throttle_value;
	int16_t        throttle_cruise;
	int16_t        throttle_deadzone;
	
	//Inertial Sensor
	vector3f_t accel_scale;
	vector3f_t accel_offset;
	vector3f_t gyro_offset;


	//AHRS


	//AttitudeControl
	acpid_t pid_rate_roll;
	acpid_t pid_rate_pitch;
	acpid_t pid_rate_yaw;

	acpid_t p_stabilize_roll;
	acpid_t p_stabilize_pitch;
	acpid_t p_stabilize_yaw;

	acpid_t p_alt_hold;
	acpid_t p_throttle_rate;
	acpid_t pid_throttle_accel;

	

}paramter_t;

//Documentation of GLobals:
typedef  union 
{
	struct 
		{
			uint8_t home_is_set         : 1; // 0	//supersimple 超级简单,无头模式
			uint8_t simple_mode         : 2; // 1,2 // This is the state of simple mode : 0 = disabled ; 1 = SIMPLE ; 2 = SUPERSIMPLE
			uint8_t pre_arm_rc_check    : 1; // 3   // true if rc input pre-arm checks have been completed successfully
			uint8_t pre_arm_check       : 1; // 4   //预解锁检测 true if all pre-arm checks (rc, accel calibration, gps lock) have been performed
			uint8_t auto_armed          : 1; // 5   // stops auto missions from beginning until throttle is raised
			uint8_t logging_started     : 1; // 6   // true if dataflash logging has started
			uint8_t land_complete       : 1; // 7   //throttle_loop true if we have detected a landing
			uint8_t new_radio_frame     : 1; // 8  // Set true if we have new PWM data to act on from the Radio,接收到控制信号
			uint8_t CH7_flag            : 2; // 9,10   // ch7 aux switch : 0 is low or false, 1 is center or true, 2 is high
			uint8_t CH8_flag            : 2; // 11,12   // ch8 aux switch : 0 is low or false, 1 is center or true, 2 is high
			uint8_t usb_connected       : 1; // 13      // true if APM is powered from USB connection
			uint8_t rc_receiver_present : 1; // 14  // true if we have an rc receiver present (i.e. if we've ever received an update
			uint8_t compass_mot         : 1; // 15  // true if we are currently performing compassmot calibration
			uint8_t motor_test          : 1; // 16  // true if we are currently performing the motors test
			uint8_t initialised         : 1; // 17  //初始化函数完成 true once the init_ardupilot function has completed.  Extended status to GCS is not sent until this completes
			uint8_t land_complete_maybe : 1; // 18  // true if we may have landed (less strict version of land_complete)
			uint8_t throttle_zero       : 1; // 19  //0油门超过一定时间 true if the throttle stick is at zero, debounced
	}flags;
  uint32_t value;
}copter_t;


////////////////////////////////////////////////////////////////////////////////
// Failsafe
////////////////////////////////////////////////////////////////////////////////
typedef struct {
	uint8_t rc_override_active : 1; // 0   // true if rc control are overwritten by ground station
	uint8_t radio : 1; // 1   // A status flag for the radio failsafe,遥控信号丢失标志
	
	uint8_t battery : 1; // 2   // A status flag for the battery failsafe
	uint8_t gps : 1; // 3   // A status flag for the gps failsafe
	uint8_t gcs : 1; // 4   // A status flag for the ground station failsafe
	uint8_t ekf : 1; // 5   // true if ekf failsafe has occurred

	int8_t radio_counter;                  // number of iterations with throttle below throttle_fs_value

	uint32_t last_heartbeat_ms;             // the time when the last HEARTBEAT message arrived from a GCS - used for triggering gcs failsafe
} failsafe_t;		//failsafe标志位


extern copter_t ap;	
extern paramter_t g;
extern failsafe_t failsafe;

void paramter_init(void);
void param_setup_and_load(void);
#endif

