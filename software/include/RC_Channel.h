
#ifndef _RC_CHANNEL_H_
#define _RC_CHANNEL_H_

#include "sys.h"


#define RC_CHANNEL_TYPE_ANGLE       0
#define RC_CHANNEL_TYPE_RANGE       1
#define RC_CHANNEL_TYPE_ANGLE_RAW   2


#define RC_MAX_CHANNELS 14



typedef struct
{
	uint8_t         type;
	
	int16_t         high;
	int16_t         low;			//输入用
	int16_t         high_out;
	int16_t         low_out;		//输出用
	

	int16_t 				radio_in;			//从接收机获取
	int16_t 				control_in;			//radio_in -->>  control_in
	
	int16_t        radio_out;
	int16_t        pwm_out;
	int16_t        servo_out;///0-1000???

	
	
	//遥控器的最小，最大和中指，电调应该经过这些值的校准
	int16_t        	radio_min;	//**		1000?
	int16_t        	radio_trim;	//**		1500?
	int16_t        	radio_max;	//**		2000?
	int16_t 		dead_zone;	//**
	int8_t          reverse;	//**
}rcchannel_t;


void rc_calc_pwm(rcchannel_t *rc);

void rc_set_angle(rcchannel_t *rc,int16_t angle);
void rc_set_range(rcchannel_t *rc,int16_t low, int16_t high);
void rc_set_type(rcchannel_t *rc,uint8_t t);
void rc_set_pwm(rcchannel_t *rc, int16_t pwm);
void rc_set_dead_zone(rcchannel_t *rc, int16_t dzone);
#endif

