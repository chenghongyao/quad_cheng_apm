#ifndef _INERTIALNAV_H_
#define _INERTIALNAV_H_
#include "sys.h"
#include "vector3f.h"
#include "AP_Buffer.h"

#define AP_INTERTIALNAV_TC_XY   2.5f // default time constant for complementary filter's X & Y axis
#define AP_INTERTIALNAV_TC_Z    3.0f //越小跟随行越好， default time constant for complementary filter's Z axis

// #defines to control how often historical accel based positions are saved
// so they can later be compared to laggy gps readings
#define AP_INTERTIALNAV_SAVE_POS_AFTER_ITERATIONS   10
#define AP_INTERTIALNAV_GPS_LAG_IN_10HZ_INCREMENTS  4       // must not be larger than size of _hist_position_estimate_x and _hist_position_estimate_y
#define AP_INTERTIALNAV_GPS_TIMEOUT_MS              300     // timeout after which position error from GPS will fall to zero



typedef struct
{
	// general variables
	vector3f_t     position_base;             // (uncorrected) position estimate in cm - relative to the home location (_base_lat, _base_lon, 0)
	vector3f_t     position_correction;       // sum of corrections to _position_base from delayed 1st order samples in cm
	vector3f_t     position;                  // sum(_position_base, _position_correction) - corrected position estimate in cm - relative to the home location (_base_lat, _base_lon, 0)位置
	vector3f_t     position_error;            // current position error in cm - is set by the check_* methods and used by update method to calculate the correction terms
	vector3f_t     accel_correction_hbf;       // horizontal body frame accelerometer corrections. here for logging purposes only
	vector3f_t     velocity;                  // latest velocity estimate (integrated from accelerometer values) in cm/s 速率
	
	float	time_constant_xy;          // time constant for horizontal corrections in s
	float	time_constant_z;           // time constant for vertical corrections in s

	
	
	struct InertialNav_flags 
	{
		uint8_t gps_glitching : 1;                // 1 if glitch detector was previously indicating a gps glitch
		uint8_t baro_glitching : 1;                // 1 if baro glitch detector was previously indicating a baro glitch
		uint8_t ignore_error : 3;                // the number of iterations for which we should ignore errors
	} flags;



	uint8_t	                xy_enabled;                // xy position estimates enabled
	float                   k1_xy;                     // gain for horizontal position correction
	float                   k2_xy;                     // gain for horizontal velocity correction
	float                   k3_xy;                     // gain for horizontal accelerometer offset correction
	uint32_t                gps_last_update;           // system time of last gps update in ms
	uint32_t                gps_last_time;             // time of last gps update according to the gps itself in ms
	uint8_t                 historic_xy_counter;       // counter used to slow saving of position estimates for later comparison to gps
	AP_BufferFloat			hist_position_estimate_x;  // buffer of historic accel based position to account for gpslag
	AP_BufferFloat			hist_position_estimate_y;  // buffer of historic accel based position to account for gps lag
	float                   lon_to_cm_scaling;         // conversion of longitude to centimeters

	
	float                   k1_z;                      // gain for vertical position correction
	float                   k2_z;                      // gain for vertical velocity correction
	float                   k3_z;                      // gain for vertical accelerometer offset correction
	uint32_t baro_last_update;
	AP_BufferFloat hist_position_estimate_z;  // buffer of historic accel based altitudes to account for barometer lag
}InertialNav_t;

extern InertialNav_t inav;

void InertialNav_init(void);
void InertialNav_udate_gains(void);
void InertialNav_update(float dt);
#endif

