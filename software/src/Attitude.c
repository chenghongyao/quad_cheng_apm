#include "Attitude.h"
#include "paramter.h"
#include "AP_Motor.h"
#include "ahrs_dcm.h"

extern int16_t desired_climb_rate;
extern float simple_cos_yaw;			// init_arm_motors中初始化为当前yaw(解锁动作调用)
extern float simple_sin_yaw;

extern int32_t super_simple_last_bearing;	// init_arm_motors中初始化为当前yaw+180庿解锁动作调用)
extern float super_simple_cos_yaw;
extern float super_simple_sin_yaw;



#define THROTTLE_IN_MIDDLE 500          // the throttle mid point
//默认参数设定500为选定油门,如果不是则对油门做补偿
int16_t get_pilot_desired_throttle(int16_t throttle_control)
{
	int16_t throttle_out;
	// exit immediately in the simple cases
	if (throttle_control == 0 || g.throttle_mid == 500)	//init_arm_motor中设置了throttle_mid??
	{
		return throttle_control;
	}

	// ensure reasonable throttle values
	throttle_control = constrain_int16(throttle_control, 0, 1000);		//限幅
	g.throttle_mid = constrain_int16(g.throttle_mid, 300, 700);			//中值限广?


	// check throttle is above, below or in the deadband死区控制
	//悬停油门大于500旿控制量会按比例提髿
	if (throttle_control < THROTTLE_IN_MIDDLE)
	{	//小于50%油门
		// below the deadband
		throttle_out = g.throttle_min + ((float)(throttle_control - g.throttle_min))*((float)(g.throttle_mid - g.throttle_min)) / ((float)(500 - g.throttle_min));
	}
	else if (throttle_control > THROTTLE_IN_MIDDLE)
	{		//大于50%油门
		// above the deadband
		throttle_out = g.throttle_mid + ((float)(throttle_control - 500))*(float)(1000 - g.throttle_mid) / 500.0f;
	}
	else
	{
		// must be in the deadband
		throttle_out = g.throttle_mid;
	}
	return throttle_out;
}

// get_pilot_desired_climb_rate - transform pilot's throttle input to
// climb rate in cm/s.  we use radio_in instead of control_in to get the full range
// without any deadzone at the bottom
#define THROTTLE_IN_DEADBAND_TOP (THROTTLE_IN_MIDDLE + g.throttle_deadzone)  // top of the deadband
#define THROTTLE_IN_DEADBAND_BOTTOM (THROTTLE_IN_MIDDLE - g.throttle_deadzone)  // bottom of the deadband
extern int16_t desired_climb_rate;		//for debug
int16_t get_pilot_desired_climb_rate(int16_t throttle_control)
{
	int16_t desired_rate = 0;
	// throttle failsafe check
	if (failsafe.radio)//失控
	{
		return 0;
	}
	// ensure a reasonable throttle value
	throttle_control = constrain_int16(throttle_control, 0, 1000);
	// ensure a reasonable deadzone
	g.throttle_deadzone = constrain_int16(g.throttle_deadzone, 0, 400);

	// check throttle is above, below or in the deadband
	//归一化得期望速率
	if (throttle_control < THROTTLE_IN_DEADBAND_BOTTOM) //在死区以下
	{
		// below the deadband,default max = 250cm/s
		desired_rate = (int32_t)g.pilot_velocity_z_max * (throttle_control - THROTTLE_IN_DEADBAND_BOTTOM) / (THROTTLE_IN_MIDDLE - g.throttle_deadzone);
	}
	else if (throttle_control > THROTTLE_IN_DEADBAND_TOP) 
	{
		// above the deadband
		desired_rate = (int32_t)g.pilot_velocity_z_max * (throttle_control - THROTTLE_IN_DEADBAND_TOP) / (THROTTLE_IN_MIDDLE - g.throttle_deadzone);
	}
	else
	{
		// must be in the deadband
		desired_rate = 0;
	}
	// desired climb rate for logging
	desired_climb_rate = desired_rate;
	return desired_rate;
}


extern void pos_control_init_takeoff(void);
void set_throttle_takeoff()
{
	// tell position controller to reset alt target and reset I terms
	pos_control_init_takeoff();
	// tell motors to do a slow start
	apmotor.flags.slow_start = 1;		
}
// get_non_takeoff_throttle - a throttle somewhere between min and mid throttle which should not lead to a takeoff
int16_t get_non_takeoff_throttle()
{
	return (g.throttle_mid / 2.0f);
}

int16_t get_throttle_pre_takeoff(int16_t throttle_control)
{
	int16_t throttle_out;
	// exit immediately if throttle_control is zero
	if (throttle_control <= 0) {
		return 0;
	}

	// sanity check throttle input
	throttle_control = constrain_int16(throttle_control, 0, 1000);

	// sanity check throttle_mid
	g.throttle_mid = constrain_int16(g.throttle_mid, 300, 700);

	//
	// sanity check throttle_min vs throttle_mid
	if (g.throttle_min > get_non_takeoff_throttle()) //保证油门在throttle_min以上
	{
		return g.throttle_min;
	}

	// check throttle is below top of deadband
	if (throttle_control < THROTTLE_IN_DEADBAND_TOP) //不起飞?
	{
		//对非起飞油门归一化
		throttle_out = g.throttle_min + ((float)(throttle_control - g.throttle_min))*((float)(get_non_takeoff_throttle() - g.throttle_min)) / ((float)(THROTTLE_IN_DEADBAND_TOP - g.throttle_min));
	}
	else
	{
		// must be in the deadband
		throttle_out = get_non_takeoff_throttle();
	}
	return throttle_out;
}


void get_pilot_desired_lean_angles(int16_t roll_in, int16_t pitch_in, int16_t *roll_out, int16_t *pitch_out)
{
	static float scaler = 1.0;
	static int16_t angle_max = 0;

	// range check the input
	roll_in = constrain_int16(roll_in, -ROLL_PITCH_INPUT_MAX, ROLL_PITCH_INPUT_MAX);			//最夿5庿
	pitch_in = constrain_int16(pitch_in, -ROLL_PITCH_INPUT_MAX, ROLL_PITCH_INPUT_MAX);

	// return filtered roll if no scaling required
	if (g.angle_max == ROLL_PITCH_INPUT_MAX)
	{
		//最大期望角度与默认一栿否则需要归一化处琿
		*roll_out = roll_in;
		*pitch_out = pitch_in;
		return;
	}

	// check if angle_max has been updated and redo scaler
	if (g.angle_max != angle_max)
	{		//更新比例系数
		angle_max = g.angle_max;
		scaler = (float)g.angle_max / (float)ROLL_PITCH_INPUT_MAX; //输入最大时的期望角庿
	}

	// convert pilot input to lean angle
	*roll_out = (int16_t)((float)roll_in * scaler);			//
	*pitch_out = (int16_t)((float)pitch_in * scaler);		//
}

float get_pilot_desired_yaw_rate(int16_t stick_angle)
{
	// convert pilot input to the desired yaw rate
	return stick_angle * g.acro_yaw_p;			//比例系数,默认4.5
}

// get_smoothing_gain - returns smoothing gain to be passed into attitude_control.angle_ef_roll_pitch_rate_ef_yaw_smooth
//      result is a number from 2 to 12 with 2 being very sluggish and 12 being very crisp
float get_smoothing_gain()//2-12
{
	return (2.0f + (float)g.rc_feel_rp / 10.0f);		//.rc_feel_rp默认100
}



// update_simple_mode - rotates pilot input if we are in simple mode
//一般模式下，control_in都认为yaw=0
//无头模式下，simple_cos_yaw起飞时记录，转换到yaw=0的位置，再转到当前位罿
void update_simple_mode(void)
{
	float rollx, pitchx;
	// exit immediately if no new radio frame or not in simple mode
	if (ap.flags.simple_mode == 0 || !ap.flags.new_radio_frame) //没有新的遥控数据，或耿飞行模式为NONE
	{
		return;
	}

	// mark radio frame as consumed
	ap.flags.new_radio_frame = 0;		//标记遥控数据已被使用,接受到遥控数据时置位
	if (ap.flags.simple_mode == 1)		//无头模式
	{//
		// rotate roll, pitch input by -initial simple heading (i.e. north facing)		//cos_yaw,sin_yaw解锁时初始化
		rollx = g.rc_1.control_in*simple_cos_yaw - g.rc_2.control_in*simple_sin_yaw;	//控制量从起飞的方向转到yaw=0的位罿
		pitchx = g.rc_1.control_in*simple_sin_yaw + g.rc_2.control_in*simple_cos_yaw;
	}
	else//超级简单模式，要有gps?
	{
		// rotate roll, pitch input by -super simple heading (reverse of heading to home)
		rollx = g.rc_1.control_in*super_simple_cos_yaw - g.rc_2.control_in*super_simple_sin_yaw;	//飞无头模??
		pitchx = g.rc_1.control_in*super_simple_sin_yaw + g.rc_2.control_in*super_simple_cos_yaw;	//
	}

	// rotate roll, pitch input from north facing to vehicle's perspective
	g.rc_1.control_in = rollx*ahrs.cos_yaw + pitchx*ahrs.sin_yaw;	//转到当前位置
	g.rc_2.control_in = -rollx*ahrs.sin_yaw + pitchx*ahrs.cos_yaw;	//
}

