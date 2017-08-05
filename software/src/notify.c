#include "notify.h"


notify_t notify;
void notify_update(void)
{
	uint8_t counter2;
	static int8_t arm_counter = 0;
	notify.boardled_counter ++;
	if(notify.boardled_counter %3 != 0)		//16Hz
	{
		return;
	}
	
	counter2 = notify.boardled_counter/3;
	if(notify.flags.initialising)			//初始化,8Hz快闪
	{
		if(counter2 & 1)	//8Hz
		{
			LEDM_ON();
		}
		else
		{
			LEDM_OFF();
		}
		
		return;
	}
	
	
	
	if(notify.flags.armed)		//解锁成功,常量
	{
		LEDM_ON();
	}
	else
	{
		if((counter2 & 0x02)==0)arm_counter++;
		
		if(notify.flags.pre_arm_check)	//预解锁检测成功,单慢闪
		{
			switch(arm_counter)
			{
				case 0:
				case 1:
				case 2:
					LEDM_ON();
					break;
				case 3:
				case 4:
				case 5:
					LEDM_OFF();
					break;
				default:
					arm_counter = -1;
					break;
			}
		}
		else//预解锁检测失败,双闪
		{
			switch(arm_counter)
			{
				case 0:
				case 1:
					LEDM_ON();
					break;
				case 2:	
					LEDM_OFF();
					break;
				case 3:
				case 4:
					LEDM_ON();
					break;
				case 5:
				case 6:
					LEDM_OFF();
					break;
				default:
					arm_counter = -1;
					break;
			}
		}
		return;
	}
	

}

