#ifndef _ATTITUDE_CONTROL_H_
#define _ATTITUDE_CONTROL_H_


#include "vector3f.h"
#include "ahrs_dcm.h"
#include "AC_PID.h"


// To-Do: change the name or move to AP_Math?
#define AC_ATTITUDE_CONTROL_DEGX100 5729.57795f                 // constant to convert from radians to centi-degrees
#define AC_ATTITUDE_CONTROL_RATE_RP_MAX_DEFAULT         18000   // maximum rotation rate in roll/pitch axis requested by angle controller used in stabilize, loiter, rtl, auto flight modes
#define AC_ATTITUDE_CONTROL_RATE_Y_MAX_DEFAULT          9000    // maximum rotation rate on yaw axis requested by angle controller used in stabilize, loiter, rtl, auto flight modes
#define AC_ATTITUDE_CONTROL_SLEW_YAW_DEFAULT            1000    // constraint on yaw angle error in degrees.  This should lead to maximum turn rate of 10deg/sed * Stab Rate P so by default will be 45deg/sec.
#define AC_ATTITUDE_CONTROL_ACCEL_RP_MAX_DEFAULT        0       // default maximum acceleration for roll/pitch axis in centi-degrees/sec/sec
#define AC_ATTITUDE_CONTROL_ACCEL_Y_MAX_DEFAULT         0       // default maximum acceleration for yaw axis in centi-degrees/sec/sec

#define AC_ATTITUDE_RATE_CONTROLLER_TIMEOUT             1.0f    // body-frame rate controller timeout in seconds
#define AC_ATTITUDE_RATE_RP_CONTROLLER_OUT_MAX          5000.0f // body-frame rate controller maximum output (for roll-pitch axis)
#define AC_ATTITUDE_RATE_YAW_CONTROLLER_OUT_MAX         4500.0f // body-frame rate controller maximum output (for yaw axis)
#define AC_ATTITUDE_ANGLE_YAW_CONTROLLER_OUT_MAX        4500.0f // earth-frame angle controller maximum output (for yaw axis)
#define AC_ATTITUDE_ANGLE_CONTROLLER_ANGLE_MAX          4500.0f // earth-frame angle controller maximum input angle (To-Do: replace with reference to aparm.angle_max)

#define AC_ATTITUDE_RATE_STAB_ROLL_OVERSHOOT_ANGLE_MAX  3000.0f // earth-frame rate stabilize controller's maximum overshoot angle
#define AC_ATTITUDE_RATE_STAB_PITCH_OVERSHOOT_ANGLE_MAX 3000.0f // earth-frame rate stabilize controller's maximum overshoot angle
#define AC_ATTITUDE_RATE_STAB_YAW_OVERSHOOT_ANGLE_MAX   1000.0f // earth-frame rate stabilize controller's maximum overshoot angle
#define AC_ATTITUDE_RATE_STAB_ACRO_OVERSHOOT_ANGLE_MAX  1000.0f // earth-frame rate stabilize controller's maximum overshoot angle

#define AC_ATTITUDE_100HZ_DT                            0.0100f // delta time in seconds for 100hz update rate
#define AC_ATTITUDE_400HZ_DT                            0.0025f // delta time in seconds for 400hz update rate

#define AC_ATTITUDE_RATE_RP_PID_DTERM_FILTER            20      // D-term filter rate cutoff frequency for Roll and Pitch rate controllers
#define AC_ATTITUDE_RATE_Y_PID_DTERM_FILTER             5       // D-term filter rate cutoff frequency for Yaw rate controller

#define AC_ATTITUDE_CONTROL_RATE_BF_FF_DEFAULT          0       // body-frame rate feedforward enabled by default


typedef struct
{

	struct AttControlFlags {
		uint8_t limit_angle_to_rate_request : 1;  // 1 if the earth frame angle controller is limiting it's maximum rate request
	} flags;



	vector3f_t            angle_ef_target;       // angle controller earth-frame targets,期望角度
	vector3f_t            rate_bf_target;        // rate controller body-frame targets
	
	vector3f_t            angle_bf_error;        // angle controller body-frame error
	//feed forward rates
	vector3f_t            rate_ef_desired;       // earth-frame feed forward rates
	vector3f_t            rate_bf_desired;       // body-frame feed forward rates
	
	acpid_t *pid_rate_roll;
	acpid_t *pid_rate_pitch;
	acpid_t *pid_rate_yaw;

	acpid_t *p_angle_roll;
	acpid_t *p_angle_pitch;
	acpid_t *p_angle_yaw;
	
	acpid_t *p_alt_hold;
	acpid_t *p_throttle_rate;
	acpid_t *pid_throttle_accel;


	float            angle_rate_rp_max;     // maximum rate request output from the earth-frame angle controller for roll and pitch axis
	float				angle_rate_y_max;      // maximum rate request output from the earth-frame angle controller for yaw axis
	float   accel_rp_max;          // maximum rotation acceleration for earth-frame roll and pitch axis
	float   accel_y_max;           // maximum rotation acceleration for earth-frame yaw axis
	int8_t  rate_bf_ff_enabled;    // Enable/Disable body frame rate feed forward


	float dt;
	
}attitude_control_t;


extern attitude_control_t atti_ctrl;


void attitude_control_init(void);
void angle_ef_roll_pitch_rate_ef_yaw_smooth(float roll_angle_ef, float pitch_angle_ef, float yaw_rate_ef, float smoothing_gain);
void rate_controller_run(void);

void relax_bf_rate_controller(void);
void set_yaw_target_to_current_heading(void);
void set_throttle_out(int16_t throttle_out,uint8_t apply_angle_boost);

#endif

