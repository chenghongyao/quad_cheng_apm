#ifndef _BAROMETER_H_
#define _BAROMETER_H_
#include "sys.h"
#include "DerivativeFilter.h"

typedef struct
{	
	uint32_t d2;
	uint16_t d2_count;
	uint32_t d1;
	uint16_t d1_count;
	
	uint8_t updated;
	uint8_t state;
	uint32_t start_time;
	uint32_t sample_dt;
	uint8_t  pressure_samples;

	float ground_pressure;
	float ground_temperature;
	float Press, Temp;
	// Internal calibration registers
	uint16_t C1, C2, C3, C4, C5, C6;
	float    D1, D2;
	uint32_t last_update;
	uint32_t last_altitude_t;
	float    altitude;

	DerivativeFilter_T climb_rate_filter;

	struct Baro_flags {
		uint8_t healthy : 1;             // true if sensor is healthy
		uint8_t alt_ok : 1;             // true if calculated altitude is ok
		uint8_t glitching : 1;
	} flags;

	//AP!
	float alt_offset;		
}barometer_t;	
extern barometer_t barometer;


void barometer_init(void);
void barometer_update(void);
void barometer_calculate(void);
void barometer_calibrate(void);
uint8_t barometer_read(void);
float barometer_get_altitude(void);
float barometer_get_climbrate(void);
#endif
