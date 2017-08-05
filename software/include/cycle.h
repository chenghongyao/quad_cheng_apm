#ifndef _CYCLE_H_
#define _CYCLE_H_
#include "sys.h"


#define CYCLE_COUNT		50

typedef enum
{
	CYCLE_CTRL_ATTIRATE=0,
	CYCLE_CTRL_ATTI,
	CYCLE_CTRL_ALTRATE,
	CYCLE_CTRL_ALT,
	CYCLE_GET_IMU,
	CYCLE_GET_ALT,
	CYCLE_SET_MOTOR,
	CYCLE_RC,
	CYCLE_UPDATE_POS,
	
	CYCLE_LOOP_100Hz,
	CYCLE_LOOP_200Hz,
	CYCLE_LOOP_500Hz,
	CYCLE_LOOP_20Hz,
}cycle_name_t;


void cycle_init(void);
float cycle_period(uint8_t n);
float cycle_times(uint8_t n);

#endif

