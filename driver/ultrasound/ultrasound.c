#include "ultrasound.h"
#include "sys.h"
#include "filter.h"
//#include "cmath.h"

#include "alt.h"





static uint8_t ultra_status = 2;
ultra_t mUltra;


//发出20us的脉冲
void ultrasound_trig()
{
//	ULTRA_TRIG = 1;
//	delay_us(20);
//	ULTRA_TRIG = 0;
	
		myputc(0x55);
		mUltra.status = 0;
}




//脉冲宽度
void ultrasound_read(uint32_t dt_impluse_us)
{
	float height;
	height = ULTRASOUND_SPEED*dt_impluse_us/(2.0*10000.0f);
	
	if(height < 350)		//超声波3.5m内有效
	{
		sensor_ultra.height = height;
		sensor_ultra.mesure_status = 1;
	}
	else
	{
		sensor_ultra.mesure_status = 2;		//数据超出范围
	}
	sensor_ultra.height_delta = sensor_ultra.height - sensor_ultra.height_old;
//	sensor_ultra.height_old = sensor_ultra.height;
}


void ultrasound_get(uint8_t dat)
{
	static uint8_t HB,LB;
	if(mUltra.status == 0)
	{
		HB = dat;
		mUltra.status = 1;
		
	}
	else if(mUltra.status == 1)
	{
		LB = dat;
		sensor_ultra.mesure_status = 1;
		mUltra.status = 2;
		sensor_ultra.height = 0.1f*(((uint16_t)HB<<8) + LB);
		sensor_ultra.height_delta = sensor_ultra.height - sensor_ultra.height_old;
		sensor_ultra.height_old = sensor_ultra.height;
		
	}
}


