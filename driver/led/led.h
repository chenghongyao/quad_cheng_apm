#ifndef __LED_H
#define __LED_H	 
#include "sys.h"




#define LED_M1	1
#define LED_M2	2
#define LED_M3	4
#define LED_M4	8


#define LED_HEAD	LED_M4|LED_M3
#define LED_TAIL	LED_M1|LED_M2
#define LED_ALL		LED_HEAD|LED_TAIL


typedef enum
{
	SHINEALL,		//全部闪烁
	SHINENONE,	//不闪烁
	SHINEHEAD,	//闪头部
	SHINETAIL,	//闪尾部
	CYCLEALL,		//流水灯
	LIGHTHEAD,	//点亮头部
	LIGHTAIL,		//点亮尾部
	LIGHTALL,		//点亮全部
}LED_MODE;

typedef enum
{
	BEEP_NONE=0,
	BEEP_SHORT,
	BEEP_LONG,
	BEEP_SHORT_TWICE,
}BEEP_MODE;


extern uint8_t ledMode;
extern uint8_t beepMode;

void led_set(uint8_t index, uint8_t sta);
void led_setMode(void);
void led_update(void);
void beep_update(void);
#endif

















