#ifndef __CONFIG_H__
#define __CONFIG_H__




#define MAIN_LOOP_RATE    100
#define MAIN_LOOP_SECONDS 0.01
#define MAIN_LOOP_MICROS  10000

//////////////////////////////////////////////////////////////////////////////
// Stabilize Rate Control
//
#ifndef ROLL_PITCH_INPUT_MAX
 # define ROLL_PITCH_INPUT_MAX      4500            // roll, pitch input range
#endif
#ifndef YAW_INPUT_MAX
# define YAW_INPUT_MAX				4500            // roll, pitch input range
#endif
#ifndef DEFAULT_ANGLE_MAX
 # define DEFAULT_ANGLE_MAX         4500            // ANGLE_MAX parameters default value
#endif


#ifndef ANGLE_RATE_MAX
 # define ANGLE_RATE_MAX            18000           // default maximum rotation rate in roll/pitch axis requested by angle controller used in stabilize, loiter, rtl, auto flight modes
#endif


// Acro Mode
#ifndef ACRO_RP_P
# define ACRO_RP_P                 4.5f
#endif

#ifndef ACRO_YAW_P
# define ACRO_YAW_P                4.5f
#endif


#ifndef ACRO_LEVEL_MAX_ANGLE
# define ACRO_LEVEL_MAX_ANGLE      3000
#endif

#ifndef ACRO_BALANCE_ROLL
#define ACRO_BALANCE_ROLL          1.0f
#endif

#ifndef ACRO_BALANCE_PITCH
#define ACRO_BALANCE_PITCH         1.0f
#endif

#ifndef ACRO_EXPO_DEFAULT
#define ACRO_EXPO_DEFAULT          0.3f
#endif

// Stabilize (angle controller) gains
#ifndef STABILIZE_ROLL_P
# define STABILIZE_ROLL_P          2.5f
#endif

#ifndef STABILIZE_PITCH_P
# define STABILIZE_PITCH_P         2.5f
#endif

#ifndef  STABILIZE_YAW_P
# define STABILIZE_YAW_P           3.5f
#endif


// PID
//
//==========ROLL速率
#ifndef RATE_ROLL_P
# define RATE_ROLL_P        		0.150f
#endif
#ifndef RATE_ROLL_I
# define RATE_ROLL_I        		0.100f
#endif
#ifndef RATE_ROLL_D
# define RATE_ROLL_D        		0.004f
#endif
#ifndef RATE_ROLL_IMAX
# define RATE_ROLL_IMAX         	1000
#endif
//==========PITCH速率
#ifndef RATE_PITCH_P
# define RATE_PITCH_P       		0.150f
#endif
#ifndef RATE_PITCH_I
# define RATE_PITCH_I       		0.100f
#endif
#ifndef RATE_PITCH_D
# define RATE_PITCH_D       		0.004f
#endif
#ifndef RATE_PITCH_IMAX
# define RATE_PITCH_IMAX        	1000
#endif
//==========YAW速率
#ifndef RATE_YAW_P
# define RATE_YAW_P              	0.200f
#endif
#ifndef RATE_YAW_I
# define RATE_YAW_I              	0.020f
#endif
#ifndef RATE_YAW_D
# define RATE_YAW_D              	0.000f
#endif
#ifndef RATE_YAW_IMAX
# define RATE_YAW_IMAX            	1000
#endif


//////////////////////////////////////////////////////////////////////////////
// Throttle control gains
//
#ifndef THROTTLE_CRUISE													//估计悬停油门
 # define THROTTLE_CRUISE       450             // default estimate of throttle required for vehicle to maintain a hover
#endif

#ifndef THR_MID_DEFAULT													//油门中值
 # define THR_MID_DEFAULT       500             // Throttle output (0 ~ 1000) when throttle stick is in mid position
#endif

#ifndef THR_MIN_DEFAULT													//最小的油门
 # define THR_MIN_DEFAULT       130             // minimum throttle sent to the motors when armed and pilot throttle above zero
#endif
#ifndef THR_MAX_DEFAULT														
 # define THR_MAX_DEFAULT       1000            // maximum throttle sent to the motors
#endif

#ifndef THR_DZ_DEFAULT			//悬停时油门死区
# define THR_DZ_DEFAULT         100             // the deadzone above and below mid throttle while in althold or loiter
#endif


//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
// Throttle Failsafe
//
#ifndef FS_THR_VALUE_DEFAULT
# define FS_THR_VALUE_DEFAULT       200	//SBUS，最小值只有350左右      975
#endif


// LAND
//
#ifndef LAND_SPEED
# define LAND_SPEED    50          // the descent speed for the final stage of landing in cm/s
#endif
#ifndef LAND_START_ALT
# define LAND_START_ALT 1000         // altitude in cm where land controller switches to slow rate of descent
#endif
#ifndef LAND_DETECTOR_TRIGGER
# define LAND_DETECTOR_TRIGGER 50    // number of 50hz iterations with near zero climb rate and low throttle that triggers landing complete.
#endif
#ifndef LAND_DETECTOR_MAYBE_TRIGGER
# define LAND_DETECTOR_MAYBE_TRIGGER   10  // number of 50hz iterations with near zero climb rate and low throttle that means we might be landed (used to reset horizontal position targets to prevent tipping over)
#endif
#ifndef LAND_DETECTOR_CLIMBRATE_MAX
# define LAND_DETECTOR_CLIMBRATE_MAX    30  // vehicle climb rate must be between -30 and +30 cm/s
#endif
#ifndef LAND_DETECTOR_BARO_CLIMBRATE_MAX
# define LAND_DETECTOR_BARO_CLIMBRATE_MAX   150  // barometer climb rate must be between -150cm/s ~ +150cm/s
#endif
#ifndef LAND_DETECTOR_DESIRED_CLIMBRATE_MAX
# define LAND_DETECTOR_DESIRED_CLIMBRATE_MAX    -20    // vehicle desired climb rate must be below -20cm/s
#endif
#ifndef LAND_DETECTOR_ROTATION_MAX
# define LAND_DETECTOR_ROTATION_MAX    0.50f   // vehicle rotation must be below 0.5 rad/sec (=30deg/sec for) vehicle to consider itself landed
#endif
#ifndef LAND_REQUIRE_MIN_THROTTLE_TO_DISARM // require pilot to reduce throttle to minimum before vehicle will disarm
# define LAND_REQUIRE_MIN_THROTTLE_TO_DISARM ENABLED
#endif
#ifndef LAND_REPOSITION_DEFAULT
# define LAND_REPOSITION_DEFAULT   1   // by default the pilot can override roll/pitch during landing
#endif
#ifndef LAND_WITH_DELAY_MS
# define LAND_WITH_DELAY_MS        4000    // default delay (in milliseconds) when a land-with-delay is triggered during a failsafe event
#endif


// Radio failsafe while using RC_override
#ifndef FS_RADIO_RC_OVERRIDE_TIMEOUT_MS
# define FS_RADIO_RC_OVERRIDE_TIMEOUT_MS  2000    // RC Radio failsafe triggers after 2 seconds while using RC_override from ground station
#endif

// Radio failsafe
#ifndef FS_RADIO_TIMEOUT_MS
#define FS_RADIO_TIMEOUT_MS            500     // RC Radio Failsafe triggers after 500 miliseconds with No RC Input
#endif


//RADIO值
#ifndef RADIO_MIN_DEFAULT
#define RADIO_MIN_DEFAULT		352+600
#endif
#ifndef RADIO_MAX_DEFAULT
#define RADIO_MAX_DEFAULT		1696+600
#endif

#ifndef RADIO_TRIM_DEFAULT
#define RADIO_TRIM_DEFAULT		1024+600
#endif

#endif

