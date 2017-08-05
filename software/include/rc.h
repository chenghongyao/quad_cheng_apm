#ifndef _RC_H_

#include "sys.h"





#define RC_DB		50							//遥控方向死区
#define RC_MAXLINE	2000-100		//通道最大值
#define RC_MINLINE	1000+100		//通道最小值
#define RC_LOSTRADIO_TIME		200		//控制信号超时时间,ms
enum 
{
	CH_THRO = 0,
	CH_YAWRATE,
	CH_ROLL,
	CH_PITCH,
	CH_GEAR,
	CH_6,
	RC_CHANNEL_NUM,
};

typedef struct
{
	uint16_t channel[RC_CHANNEL_NUM];
	uint16_t channel_old[RC_CHANNEL_NUM];
	float roll;
	float pitch;
	float yaw_rate;
	float thro;
	
	uint8_t recv;
	uint32_t recv_last_time;
	uint8_t get_signal;
}rc_t;

extern rc_t mRC;

void rc_update(void);
void rc_init(void);
void rc_dealData(float dt);



#endif

