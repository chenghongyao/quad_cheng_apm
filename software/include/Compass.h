#ifndef _COMPASS_H_
#define _COMPASS_H_
#include "sys.h"
#include "matrix3f.h"

typedef struct
{
	vector3f_t field;
	uint32_t last_update_time;
	
	vector3f_t scale;
	vector3f_t offset;
	float	declination;
	uint8_t initialised;
}Compass_t;


extern Compass_t compass;

uint8_t Compass_read(void);
void Compass_init(void);
	
float Compass_calculate_heading(const matrix3f_t *dcm_mat);
#endif
