#ifndef _ULTRASOUND_H
#define _ULTRASOUND_H
#include "sys.h"



#define ULTRASOUND_SPEED	340.0f		//声音速度,m/s


typedef struct 
{	
	float distance_raw;	//超声波原始距离,m
	float distance;			//超声波滤波距离,m
	float speed;				//超声波测量差分速度
	float accel;				//超声波测量差分加速度
	
	
	float distance_pre;
	float speed_pre;
	uint8_t IsUltraUpdated;
	
	

	uint8_t status;
}ultra_t;

extern ultra_t mUltra;

void ultrasound_trig(void);
void ultrasound_read(uint32_t dt_impluse_us);
void ultrasound_get(uint8_t dat);
#endif
