#ifndef _AP_MOTOR_H_
#define _AP_MOTOR_H_

#include "sys.h"
#include "RC_Channel.h"
#include "AP_Curve.h"



#define AP_MOTORS_MAX_NUM_MOTORS 				8

#define AP_MOTORS_SPIN_WHEN_ARMED				0		//怠速转动值,spin motors at this PWM value when armed
#define AP_MOTOR_SLOW_START_INCREMENT           10      // max throttle ramp speed (i.e. motors can reach full throttle in 1 second)
#define AP_MOTOR_SLOW_START_LOW_END_INCREMENT   2       // min throttle ramp speed (i.e. motors will speed up from zero to _spin_when_armed speed in about 1 second)
#define AP_MOTORS_MATRIX_YAW_LOWER_LIMIT_PWM    200


//THROTTLE
#define AP_MOTORS_DEFAULT_MIN_THROTTLE  130
#define AP_MOTORS_DEFAULT_MID_THROTTLE  500
#define AP_MOTORS_DEFAULT_MAX_THROTTLE  1000

//THROTTLE_CURVE
#define THROTTLE_CURVE_ENABLED      1   // throttle curve disabled by default
#define THROTTLE_CURVE_MID_THRUST   52  // throttle which produces 1/2 the maximum thrust.  expressed as a percentage of the full throttle range (i.e 0 ~ 100)
#define THROTTLE_CURVE_MAX_THRUST   93  // throttle which produces the maximum thrust.  expressed as a percentage of the full throttle range (i.e 0 ~ 100)

// offsets for motors in motor_out, _motor_filtered and _motor_to_channel_map arrays
#define AP_MOTORS_MOT_1 0
#define AP_MOTORS_MOT_2 1
#define AP_MOTORS_MOT_3 2
#define AP_MOTORS_MOT_4 3
#define AP_MOTORS_MOT_5 4
#define AP_MOTORS_MOT_6 5
#define AP_MOTORS_MOT_7 6
#define AP_MOTORS_MOT_8 7


#define AP_MOTORS_MATRIX_YAW_FACTOR_CW   -1
#define AP_MOTORS_MATRIX_YAW_FACTOR_CCW   1


#define ARM_DELAY               20  // called at 10hz so 2 seconds
#define DISARM_DELAY            20  // called at 10hz so 2 seconds
#define AUTO_TRIM_DELAY         100 // called at 10hz so 10 seconds
#define AUTO_DISARMING_DELAY    15  // called at 1hz so 15 seconds

typedef struct
{
	
	struct AP_Motors_limit 			//在apmotor_output_armed中更新
	{
		uint8_t roll_pitch      : 1; // we have reached roll or pitch limit ,!!!!!
		uint8_t yaw             : 1; // we have reached yaw limit
		uint8_t throttle_lower  : 1; // we have reached throttle's lower limit
		uint8_t throttle_upper  : 1; // we have reached throttle's upper limit
	}limit;		//???// initialize limits flag in output_armed()
		 
	struct AP_Motors_flags 
	{
		uint8_t armed               : 1;    // 1 if the motors are armed, 0 if disarmed
		uint8_t frame_orientation   : 4;    // PLUS_FRAME 0, X_FRAME 1, V_FRAME 2, H_FRAME 3, NEW_PLUS_FRAME 10, NEW_X_FRAME, NEW_V_FRAME, NEW_H_FRAME
		uint8_t slow_start          : 1;    // 1 if slow start is active
		uint8_t slow_start_low_end  : 1;    // 1 just after arming so we can ramp up the spin_when_armed value
	} flags;
	
	
	

	int16_t 			spin_when_armed_ramped;
	int16_t				spin_when_armed;       // used to control whether the motors always spin when armed.  pwm value above radio_min


	int16_t             min_throttle;          //default=130 防止电机死区停转,the minimum throttle to be sent to the motors when they're on (prevents motors stalling while flying)
	int16_t             max_throttle;          //default=1000 (0-1000)the maximum throttle to be sent to the motors (sometimes limited by slow start)
	int16_t             hover_out;             //default=500 the estimated hover throttle in pwm (i.e. 1000 ~ 2000).  calculated from the THR_MID parameter
		
	int16_t angle_boost;
	int16_t rpy_out[AP_MOTORS_MAX_NUM_MOTORS];		 // buffer so we don't have to multiply coefficients multiple times.
	int16_t motor_out[AP_MOTORS_MAX_NUM_MOTORS];    // final outputs sent to the motors
	//使用removed_motor,add_motor初始化
	uint8_t		motor_enabled[AP_MOTORS_MAX_NUM_MOTORS];
	float       roll_factor[AP_MOTORS_MAX_NUM_MOTORS]; // each motors contribution to roll
	float       pitch_factor[AP_MOTORS_MAX_NUM_MOTORS]; // each motors contribution to pitch
	float       yaw_factor[AP_MOTORS_MAX_NUM_MOTORS];  // each motors contribution to yaw (normally 1 or -1)


	//电机输出曲线
	AP_CurveInt16_Size4_T throttle_curve;
	int8_t             throttle_curve_enabled; // enable throttle curve
	int8_t             throttle_curve_mid;    // throttle which produces 1/2 the maximum thrust.  expressed as a percentage (i.e. 0 ~ 100 ) of the full throttle range
	int8_t             throttle_curve_max;    // throttle which produces the maximum thrust.  expressed as a percentage (i.e. 0 ~ 100 ) of the full throttle range
	
	


	//!!!!!
	rcchannel_t         *rc_roll;
	rcchannel_t         *rc_pitch;
	rcchannel_t         *rc_yaw;
	rcchannel_t         *rc_throttle;
	
	
}apmotor_t;

extern apmotor_t apmotor;

void apmotor_init(rcchannel_t *rc_roll, rcchannel_t *rc_pitch, rcchannel_t *rc_throttle, rcchannel_t *rc_yaw);
void apmotor_ouput_min(void);
uint8_t arm_checks(uint8_t display_failure);
uint8_t init_arm_motors(void);
void init_disarm_motors(void);
void apmotor_output(void);
int16_t apmotor_get_angle_boost(int16_t throttle_pwm);
void pre_arm_checks(uint8_t display_failure);
#endif

