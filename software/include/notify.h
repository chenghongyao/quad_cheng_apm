#ifndef _NOTIFY_H_
#define _NOTIFY_H_

#include "sys.h"

typedef struct
{
	
	struct 
	{
			uint16_t initialising       : 1;    // 1 if initialising and copter should not be moved
			
			uint16_t gps_status         : 3;    // 0 = no gps, 1 = no lock, 2 = 2d lock, 3 = 3d lock, 4 = dgps lock, 5 = rtk lock
			uint16_t gps_glitching      : 1;    // 1 if gps position is not good
			uint16_t baro_glitching     : 1;    // 1 if baro altitude is not good
			
			uint16_t pre_arm_check      : 1;    // 0 = failing checks, 1 = passed
			uint16_t arming_failed      : 1;    // 1 if copter failed to arm after user input
			uint16_t armed              : 1;    // 0 = disarmed, 1 = armed
		
			uint16_t pre_arm_gps_check  : 1;    // 0 = failing pre-arm GPS checks, 1 = passed
			uint16_t save_trim          : 1;    // 1 if gathering trim data
			uint16_t esc_calibration    : 1;    // 1 if calibrating escs
		
			uint16_t failsafe_radio     : 1;    // 1 if radio failsafe
			uint16_t failsafe_battery   : 1;    // 1 if battery failsafe
			uint16_t failsafe_gps       : 1;    // 1 if gps failsafe
		
			uint16_t parachute_release  : 1;    // 1 if parachute is being released
			uint16_t ekf_bad            : 1;    // 1 if ekf is reporting problems

			// additional flags
			uint16_t external_leds      : 1;    // 1 if external LEDs are enabled (normally only used for copter)
	}flags;
	
	uint16_t boardled_counter;
}notify_t;

extern notify_t notify;

void notify_update(void);

#endif

