#ifndef _MOTOR_H
#define _MOTOR_H
#include "sys.h"


//#define MOT_PERIOC_DUTY	20000				//脉冲周期计数值
//#define MOT_TO_PWMDUTY		((float)MOT_PERIOC_DUTY/20000.0f)			//20000us的PWM计数值/20000us

void motor_set(uint16_t MHL,uint16_t MHR,uint16_t MTL,uint16_t MTR);
void motor_write(uint8_t index,uint16_t value);
void motor_init(void);
#endif
