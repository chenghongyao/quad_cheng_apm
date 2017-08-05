#ifndef _DEFINES_H
#define _DEFINES_H



// Radio failsafe definitions (FS_THR parameter)
#define FS_THR_DISABLED                    0
#define FS_THR_ENABLED_ALWAYS_RTL          1
#define FS_THR_ENABLED_CONTINUE_MISSION    2
#define FS_THR_ENABLED_ALWAYS_LAND         3


	
// Arming Check Enable/Disable bits
#define ARMING_CHECK_NONE                   0x00
#define ARMING_CHECK_ALL                    0x01
#define ARMING_CHECK_BARO                   0x02
#define ARMING_CHECK_COMPASS                0x04
#define ARMING_CHECK_GPS                    0x08
#define ARMING_CHECK_INS                    0x10
#define ARMING_CHECK_PARAMETERS             0x20
#define ARMING_CHECK_RC                     0x40
#define ARMING_CHECK_VOLTAGE                0x80



// RC Feel roll/pitch definitions
#define RC_FEEL_RP_VERY_SOFT        0
#define RC_FEEL_RP_SOFT             25
#define RC_FEEL_RP_MEDIUM           50
#define RC_FEEL_RP_CRISP            75
#define RC_FEEL_RP_VERY_CRISP       100


#endif
