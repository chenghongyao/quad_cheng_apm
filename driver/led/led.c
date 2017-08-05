#include "led.h"

#include "flight.h"
#include "imu.h"


uint8_t ledMode=SHINEALL;
uint8_t beepMode = BEEP_NONE;

void led_set(uint8_t index,uint8_t sta)
{
	ledM = !sta;
}

void led_update()
{
	static uint8_t ledstatus = 0;
	static uint8_t ledbuf = LED_M1;
	led_setMode();
	switch(ledMode)
	{
		case SHINEALL:
			led_set(LED_ALL,ledstatus);
			break;
		case SHINENONE:
			led_set(LED_ALL,0);
			break;
		case SHINEHEAD:
			led_set(LED_HEAD,ledstatus);
			led_set(LED_TAIL,0);
			break;
		case SHINETAIL:
			led_set(LED_HEAD,0);
			led_set(LED_TAIL,ledstatus);
			break;
		case CYCLEALL:
			led_set(LED_ALL,0);
			led_set(ledbuf,1);			
			if(ledbuf==LED_M4)
				ledbuf = LED_M1;
			else 
				ledbuf<<=1;
			break;
		case LIGHTHEAD:
			led_set(LED_HEAD,1);
			led_set(LED_TAIL,0);
		break;
		case LIGHTAIL:
			led_set(LED_HEAD,0);
			led_set(LED_TAIL,1);
		break;
		case LIGHTALL:
			led_set(LED_ALL,1);
		break;
		default:
			break;
	}
	ledstatus = !ledstatus;
}

#define BEEP_TIMES_SHORT2	1												//短鸣两声时间
#define BEEP_TIMES_SHORT	2												//短鸣时间
#define BEEP_TIMES_LONG		(2*BEEP_TIMES_SHORT)		//长鸣时间
void beep_update()
{
	static uint16_t cnt;
	static uint8_t step=0;
	static uint8_t last_mode = BEEP_NONE;
	
	if(last_mode == BEEP_NONE)		//等待上一个命令执行完成
	{
		last_mode = beepMode;
		beepMode = BEEP_NONE;
	}	

	switch(last_mode)
	{
		case BEEP_NONE:
			beepM = 0;
			cnt = 0;
			break;
		case BEEP_SHORT:				//短鸣
			beepM = 1;
			if(++cnt>BEEP_TIMES_SHORT)
			{
				beepM = 0;
				last_mode = BEEP_NONE;
				cnt = 0;
			}
			break;
		case BEEP_LONG:					//长鸣
			beepM = 1;
			if(++cnt>BEEP_TIMES_LONG)
			{
				beepM = 0;
				last_mode = BEEP_NONE;
				cnt = 0;
			}
			break;
		case BEEP_SHORT_TWICE:	//短鸣两声		
			switch(step)
			{
				case 0:
					beepM = 1;
					if(++cnt>BEEP_TIMES_SHORT2) 
					{
						beepM = 0;
						step++;
						cnt = 0;
					}
					break;
			case 1:
				beepM = 0;
				if(++cnt>BEEP_TIMES_SHORT2)
				{
					step++;
					cnt = 0;
				}
				break;
			case 2:
				beepM = 1;
				if(++cnt>BEEP_TIMES_SHORT2)
				{
					beepM = 0;
					step=0;
					last_mode = BEEP_NONE;
					cnt = 0;
				}
				break;
				default:
						beepM = 0;
						step=0;
						cnt = 0;
					break;
			}
			break;
		default:
			beepM = 0;
			step=0;
			last_mode = BEEP_NONE;
			cnt = 0;
		break;
	}
	
}

/////////////////////////////////////////////////////
//校准中->CYCLEALL
//丢失信号->LIGHTHEAD
//正常->SHINEHEAD
//越往下优先级越高
////////////////////////////////////////////////////
void led_setMode()
{
	if(mImu.acc_w_ready == 0 || mImu.mag_ready == 0)
	{
		ledMode = SHINEALL;
	}
	else if(mFlight.flyEnable)
	{
		ledMode = SHINEALL;
	}
	else
	{
		ledMode = LIGHTALL;
	}
}
