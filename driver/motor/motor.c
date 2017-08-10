#include "motor.h"
#include "sys.h"

#include "AP_Motor.h"

#include "paramter.h"

uint16_t motor_monitor[4];
void motor_set(uint16_t MHL,uint16_t MHR,uint16_t MTL,uint16_t MTR)
{
	MOTOR_HL = 1000+MHL;
	MOTOR_HR = 1000+MHR;
	MOTOR_TL = 1000+MTL;
	MOTOR_TR = 1000+MTR;
}



void motor_write(uint8_t index,uint16_t value)
{
	//value = 0;
	switch(index)
	{
		case AP_MOTORS_MOT_1:
			motor_monitor[AP_MOTORS_MOT_1] = value-g.rc_3.radio_min;
			break;
		case AP_MOTORS_MOT_2:
			motor_monitor[AP_MOTORS_MOT_2] = value-g.rc_3.radio_min;
			break;
		case AP_MOTORS_MOT_3:
			motor_monitor[AP_MOTORS_MOT_3] = value-g.rc_3.radio_min;
			break;
		case AP_MOTORS_MOT_4:
			motor_monitor[AP_MOTORS_MOT_4] = value-g.rc_3.radio_min;
			break;
		case AP_MOTORS_MOT_5:
			break;
		case AP_MOTORS_MOT_6:
			break;
		case AP_MOTORS_MOT_7:
			break;
		case AP_MOTORS_MOT_8:
			break;
		default:
			break;
	}
	

#if 1
	switch(index)
	{
		case AP_MOTORS_MOT_1:
			MOTOR_HR = value;
			break;
		case AP_MOTORS_MOT_2:
			MOTOR_TL = value;
			break;
		case AP_MOTORS_MOT_3:
			MOTOR_HL = value;
			break;
		case AP_MOTORS_MOT_4:
			MOTOR_TR = value;
			break;
		case AP_MOTORS_MOT_5:
			break;
		case AP_MOTORS_MOT_6:
			break;
		case AP_MOTORS_MOT_7:
			break;
		case AP_MOTORS_MOT_8:
			break;
		default:
			break;
	}
#endif
}


//#define MOTOR_INIT_ESC
void motor_init()
{
#ifdef	MOTOR_INIT_ESC
	motor_set(1000,1000,1000,1000);
	delay_ms(1000);
	delay_ms(1000);
	delay_ms(1000);
#endif	
	motor_set(0,0,0,0);
}



