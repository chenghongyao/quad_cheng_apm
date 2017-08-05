#include "delay.h"


static uint32_t usTicks = 72;		//72M=每us72次
static uint32_t sysTickUptime = 0;


/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
// SysTick
void SysTick_Handler(void)
{
    sysTickUptime++;
}



//
void delay_ms(uint16_t nms)
{
		uint32_t t0=micros();
		while(micros() - t0 < nms * 1000);
			
}
void delay_us(uint32_t nus)
{
		volatile uint32_t t0 = micros();
		while((micros() - t0) < nus);
			
}

// Return system uptime in microseconds (rollover in 70minutes)
//返回 us
uint32_t micros(void)
{
    register uint32_t ms, cycle_cnt;
    do {
        ms = sysTickUptime;
        cycle_cnt = SysTick->VAL;
    } while (ms != sysTickUptime);
    return (ms * 1000) + (usTicks * 1000 - cycle_cnt) / usTicks;
}

// Return system uptime in milliseconds (rollover in 49 days)
uint32_t millis(void)
{
    return sysTickUptime;
}


void delay_init()
{
	usTicks = SystemCoreClock/1000000;				//每us时钟数
	SysTick_Config(usTicks*1000);							//1ms定时中断用于计时
}
