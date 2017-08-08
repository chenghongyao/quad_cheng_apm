#ifndef _POSCONTROL_H_
#define _POSCONTROL_H_
#include "sys.h"
#include "vector3f.h"
#include "matrix3f.h"
#include "AC_PID.h"
#include "LowPassFilter.h"




#define POSCONTROL_TAKEOFF_JUMP_CM              0.0f  // during take-off altitude target is set to current altitude + this value

#define POSCONTROL_SPEED                        500.0f  // default horizontal speed in cm/s
#define POSCONTROL_SPEED_DOWN                  -150.0f  // default descent rate in cm/s
#define POSCONTROL_SPEED_UP                     250.0f  // default climb rate in cm/s
#define POSCONTROL_VEL_XY_MAX_FROM_POS_ERR      200.0f  // max speed output from pos_to_vel controller when feed forward is used


#define POSCONTROL_ACTIVE_TIMEOUT_MS            200     // position controller is considered active if it has been called within the past 200ms (0.2 seconds)
#define POSCONTROL_ACCELERATION_MIN             50.0f   // minimum horizontal acceleration in cm/s/s - used for sanity checking acceleration in leash length calculation
#define POSCONTROL_LEASH_LENGTH_MIN             100.0f  // minimum leash lengths in cm

#define POSCONTROL_ACCEL_Z                      250.0f  // default vertical acceleration in cm/s/s.
#define POSCONTROL_ACCEL_Z_DTERM_FILTER         20      // Z axis accel controller's D term filter (in hz)

#define POSCONTROL_VEL_ERROR_CUTOFF_FREQ        4.0     // 4hz low-pass filter on velocity error
#define POSCONTROL_ACCEL_ERROR_CUTOFF_FREQ      2.0     // 2hz low-pass filter on accel error


typedef struct
{
	vector3f_t pos_target;
	vector3f_t vel_desired;
	vector3f_t pos_error;
	vector3f_t accel_error;

	vector3f_t vel_target;            // velocity target in cm/s calculated by pos_to_rate step
	vector3f_t accel_feedforward;     // feedforward acceleration in cm/s/s
	vector3f_t vel_last;
	vector3f_t vel_error;

	float throttle_hover;
	float       alt_max;               // max altitude - should be updated from the main code with altitude limit from fence
	uint32_t last_update_z_ms;
	float		dt;
	float       speed_down_cms;        // max descent rate in cm/s
	float       speed_up_cms;          // max climb rate in cm/s
	float       speed_cms;             // max horizontal speed in cm/s
	float       accel_z_cms;           // max vertical acceleration in cm/s/s
	float       accel_cms;             // max horizontal acceleration in cm/s/s
	float       leash_down_z;          // 下降高度误差限制vertical leash down in cm.  target will never be further than this distance below the vehicle
	float       leash_up_z;            // 上升高度误差限制vertical leash up in cm.  target will never be further than this distance above the vehicle
	
	low_pass_fiter_t vel_error_filter;
	low_pass_fiter_t accel_error_filter;

	acpid_t *p_alt_pos;
	acpid_t *p_alt_rate;
	acpid_t *pid_alt_accel;


	// general purpose flags
	struct poscontrol_flags {
		uint8_t recalc_leash_z : 1;    // 1 if we should recalculate the z axis leash length
		uint8_t recalc_leash_xy : 1;    // 1 if we should recalculate the xy axis leash length
		uint8_t force_recalc_xy : 1;    // 1 if we want the xy position controller to run at it's next possible time.  set by loiter and wp controllers after they have updated the target position.
		uint8_t slow_cpu : 1;    // 1 if we are running on a slow_cpu machine.  xy position control is broken up into multiple steps
		uint8_t keep_xy_I_terms : 1;    // 1 if we are to keep I terms when the position controller is next run.  Reset automatically back to zero in update_xy_controller.
		uint8_t reset_desired_vel_to_pos : 1;    // 1 if we should reset the rate_to_accel_xy step
		uint8_t reset_rate_to_accel_xy : 1;    // 1 if we should reset the rate_to_accel_xy step
		uint8_t reset_rate_to_accel_z : 1;    // 1 if we should reset the rate_to_accel_z step
		uint8_t reset_accel_to_throttle : 1;    // 1 if we should reset the accel_to_throttle step of the z-axis controller
		uint8_t freeze_ff_xy : 1;    // 1 use to freeze feed forward during step updates
		uint8_t freeze_ff_z : 1;    // 1 use to freeze feed forward during step updates
	} flags;

	// limit flags structure
	struct poscontrol_limit_flags {
		uint8_t pos_up : 1;    // 1 if we have hit the vertical position leash limit while going up
		uint8_t pos_down : 1;    // 1 if we have hit the vertical position leash limit while going down
		uint8_t vel_up : 1;    // 1 if we have hit the vertical velocity limit going up
		uint8_t vel_down : 1;    // 1 if we have hit the vertical velocity limit going down
		uint8_t accel_xy : 1;    // 1 if we have hit the horizontal accel limit
	} limit;


}pos_control_t;
extern pos_control_t pos_ctrl;
void pos_control_init(void);
void pos_control_init_takeoff(void);
void pos_control_set_alt_target_from_climb_rate(float climb_rate_cms, float dt, uint8_t force_descend);
void pos_control_update_z_controller(void);
void pos_control_set_dt(float dt);
#endif
