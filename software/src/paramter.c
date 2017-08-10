#include "paramter.h"
#include "sys.h"
#include "AP_InertialSensor.h"
#include "AC_PID.h"

#include "config.h"
#include "defines.h"





void param_setup_and_load()
{


	//Inertial Sensor
	//vector3f_copy(&g.accel_scale,&ins.accel_scale);
	//vector3f_copy(&g.accel_offset,&ins.accel_offset);
	//vector3f_copy(&g.gyro_offset,&ins.gyro_offset);

	//AHRS_DCM

	//RC:遥控器校准值//
	//roll
	g.rc_1.radio_min = RADIO_MIN_DEFAULT;
	g.rc_1.radio_max = RADIO_MAX_DEFAULT;
	g.rc_1.radio_trim = RADIO_TRIM_DEFAULT;		//中值
	g.rc_1.dead_zone = 30;			//
	g.rc_1.reverse = 1;				//
	//pitch
	g.rc_2.radio_min = RADIO_MIN_DEFAULT;
	g.rc_2.radio_max = RADIO_MAX_DEFAULT;
	g.rc_2.radio_trim = RADIO_TRIM_DEFAULT;		//中值
	g.rc_2.dead_zone = 30;			//
	g.rc_2.reverse = 1;				//

	//throttle!!!
	g.rc_3.radio_min = RADIO_MIN_DEFAULT;
	g.rc_3.radio_max = RADIO_MAX_DEFAULT;
	g.rc_3.radio_trim = RADIO_TRIM_DEFAULT;		//中值
	g.rc_3.dead_zone = 30;			//
	g.rc_3.reverse = 1;				//

	//yaw
	g.rc_4.radio_min = RADIO_MIN_DEFAULT;
	g.rc_4.radio_max = RADIO_MAX_DEFAULT;
	g.rc_4.radio_trim = RADIO_TRIM_DEFAULT;		//中值
	g.rc_4.dead_zone = 40;			//
	g.rc_4.reverse = 1;				//

	g.rc_5.radio_min = RADIO_MIN_DEFAULT;
	g.rc_5.radio_max = RADIO_MAX_DEFAULT;
	g.rc_5.radio_trim = RADIO_MIN_DEFAULT;
	g.rc_5.dead_zone = 0;			//
	g.rc_5.reverse = 1;				//

	g.rc_6.radio_min = RADIO_MIN_DEFAULT;
	g.rc_6.radio_max = RADIO_MAX_DEFAULT;
	g.rc_6.radio_trim = RADIO_MIN_DEFAULT;
	g.rc_6.dead_zone = 0;			//
	g.rc_6.reverse = 1;				//
	
	g.rc_7.radio_min = RADIO_MIN_DEFAULT;
	g.rc_7.radio_max = RADIO_MAX_DEFAULT;
	g.rc_7.radio_trim = RADIO_MIN_DEFAULT;
	g.rc_7.dead_zone = 0;			//
	g.rc_7.reverse = 1;				//

	g.rc_8.radio_min = RADIO_MIN_DEFAULT;
	g.rc_8.radio_max = RADIO_MAX_DEFAULT;
	g.rc_8.radio_trim = RADIO_MIN_DEFAULT;
	g.rc_8.dead_zone = 0;			//
	g.rc_8.reverse = 1;				//

/////// Throttle////////////////////////////////////////////////////////////////////////
	g.throttle_min = THR_MIN_DEFAULT;			//radio转为最小最大油门
	g.throttle_max = THR_MAX_DEFAULT ;
	g.throttle_mid = THR_MID_DEFAULT;			//悬停油门，改值改变时，油门输出量会做比例修改

	g.failsafe_throttle = FS_THR_DISABLED;				//FsySky接收机才有的功能????
	g.failsafe_throttle_value = FS_THR_VALUE_DEFAULT;
	g.throttle_cruise = THROTTLE_CRUISE;
	g.throttle_deadzone = THR_DZ_DEFAULT;			//高于最小值的死区
	

	//自稳模式,PID
	acpid_set(&g.p_stabilize_roll, STABILIZE_ROLL_P, 0, 0, 0);
	acpid_set(&g.p_stabilize_pitch, STABILIZE_PITCH_P, 0, 0, 0);
	acpid_set(&g.p_stabilize_yaw, STABILIZE_YAW_P, 0, 0, 0);

	acpid_set(&g.pid_rate_roll, RATE_ROLL_P, RATE_ROLL_I, RATE_ROLL_D, RATE_ROLL_IMAX);
	acpid_set(&g.pid_rate_pitch, RATE_PITCH_P, RATE_PITCH_I, RATE_PITCH_D, RATE_PITCH_IMAX);
	acpid_set(&g.pid_rate_yaw, RATE_YAW_P, RATE_YAW_I, RATE_YAW_D, RATE_YAW_IMAX);

	//定高PID
	acpid_set(&g.p_alt_pos, ALT_HOLD_P, 0, 0, 0);	//default = 1.0
	acpid_set(&g.p_alt_rate, THROTTLE_RATE_P, 0, 0, 0);
	acpid_set(&g.pid_alt_accel, THROTTLE_ACCEL_P, THROTTLE_ACCEL_I, THROTTLE_ACCEL_D, THROTTLE_ACCEL_IMAX);


	//g,其他
	g.acro_yaw_p = ACRO_YAW_P;
	g.angle_max = DEFAULT_ANGLE_MAX;
	g.arming_check = ARMING_CHECK_ALL;		//解锁时检测使能
	g.rc_feel_rp = RC_FEEL_RP_VERY_CRISP;	//100
	g.esc_calibrate = 0;
	//定高参数
	g.pilot_velocity_z_max = PILOT_VELZ_MAX;


	//
	g.flight_modes[0] = STABILIZE;
	g.flight_modes[1] = STABILIZE;
	g.flight_modes[2] = ALT_HOLD;
	g.pilot_velocity_z_max = PILOT_VELZ_MAX;
	g.pilot_accel_z = PILOT_ACCEL_Z_DEFAULT;

	
	//ap////////////////////////////////////////////
	ap.flags.pre_arm_check = 0;		//
	ap.flags.pre_arm_rc_check = 0;
	ap.flags.simple_mode = 0;		//0 =自稳 ; 1 = SIMPLE(无头); 2 = SUPERSIMPLE
	ap.flags.land_complete = 1;
	ap.flags.land_complete_maybe = 1;
	
	
	//
	failsafe.radio = 0;
	failsafe.radio_counter = 0;
}


