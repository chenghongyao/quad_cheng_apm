#include "land_detector.h"
#include "paramter.h"
#include "AP_InertialSensor.h"
#include "InertialNav.h"
#include "PosControl.h"
#include "AP_Motor.h"
#include "ahrs_dcm.h"
#include "AP_State.h"
#include "cmath.h"
#include "Attitude.h"


// counter to verify landings
static uint16_t land_detector = LAND_DETECTOR_TRIGGER;  // we assume we are landed



uint8_t land_complete_maybe(void)
{
    return (ap.flags.land_complete || ap.flags.land_complete_maybe);
}



extern float baro_climbrate;
void update_land_detector(void)
{
	uint8_t climb_rate_low = (ABS(inav.velocity.z) < LAND_DETECTOR_CLIMBRATE_MAX) && (ABS(baro_climbrate) < LAND_DETECTOR_BARO_CLIMBRATE_MAX);
	uint8_t target_climb_rate_low = (pos_ctrl.vel_desired.z <= LAND_DETECTOR_DESIRED_CLIMBRATE_MAX);//期望速率小于,摇杆拉低在中位以下
	uint8_t motor_at_lower_limit = apmotor.limit.throttle_lower;		//油门达到下限
	uint8_t throttle_low = (apmotor.rc_throttle->servo_out < get_non_takeoff_throttle());//油门输出小于非起飞油门(mid/2??)
	uint8_t not_rotating_fast = (vector3f_length(&ahrs.omega) < LAND_DETECTOR_ROTATION_MAX);		//旋转速度小于落地最大旋转速度30deg/sec

	if (climb_rate_low && target_climb_rate_low && motor_at_lower_limit && throttle_low && not_rotating_fast) 
	{//确认降落了
		if (!ap.flags.land_complete) 
		{//避免重复设置
			// increase counter until we hit the trigger then set land complete flag
			if (land_detector < LAND_DETECTOR_TRIGGER) 
			{		//检测该条件一定时间
				land_detector++;
			}
			else
			{//确定降落了
				set_land_complete(1);//设置降落标志
				land_detector = LAND_DETECTOR_TRIGGER;
			}
		}
	}
	else
	{
		// we've sensed movement up or down so reset land_detector
		land_detector = 0;
		// if throttle output is high then clear landing flag
		if (apmotor.rc_throttle->servo_out > get_non_takeoff_throttle())
		{//油门大于起飞油门
			set_land_complete(0);		//设置没有降落标志
		}
	}

	// set land maybe flag
	set_land_complete_maybe(land_detector >= LAND_DETECTOR_MAYBE_TRIGGER);		//可能要降落
}
