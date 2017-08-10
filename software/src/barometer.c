#include "barometer.h"
#include "ms5611.h"
#include "Scheduler.h"
#include "delay.h"


void barometer_init(void)
{
	barometer.d2 = 0;
	barometer.d2_count = 0;
	barometer.d1 = 0;
	barometer.d1_count = 0;
	barometer.updated = 0;
	DerivativeFilter_Init(&barometer.climb_rate_filter, 7);

	
	myiic_sem_take_blocking();
	ms5611_reset();
	delay_ms(10);	
	ms5611_readPROM(&barometer.C1, &barometer.C2, &barometer.C3, &barometer.C4, &barometer.C5, &barometer.C6);//读取PROM
	delay_ms(100);
	ms5611_start_T();		//启动温度测量
	myiic_sem_give();
	
	
	barometer.state = 0;
	barometer.start_time = micros();
	scheduler_suspend_timer_procs();
	scheduler_register_timer_process(barometer_update);
	scheduler_resume_timer_procs();
	
	barometer.flags.glitching  = 0;
	
	barometer_calibrate();
}


void barometer_calculate()
{
	int32_t off2 = 0, sens2 = 0, delt;
	int32_t temperature,pressure;
	int32_t dT;
	int64_t off;
	int64_t sens;
	float alt_3;
	dT =  barometer.D2 - ((uint32_t) barometer.C5 << 8);		//???
	off = ((uint32_t) barometer.C2 << 16) + (((int64_t)dT *  barometer.C4) >> 7);
	sens = ((uint32_t) barometer.C1 << 15) + (((int64_t)dT *  barometer.C3) >> 8);
	temperature = 2000 + (((int64_t)dT *  barometer.C6) >> 23);					//

	if (temperature < 2000)// temperature lower than 20degC 
	{ 
			delt = temperature - 2000;
			delt = delt * delt;
			off2 = (5 * delt) >> 1;
			sens2 = (5 * delt) >> 2;
			if (temperature < -1500) // temperature lower than -15degC
			{ 
					delt = temperature + 1500;
					delt = delt * delt;
					off2  += 7 * delt;
					sens2 += (11 * delt) >> 1;
			}
	}
	off  -= off2; 
	sens -= sens2;
	barometer.Press = pressure = ((((uint32_t)barometer.D1 * sens ) >> 21) - off) >> 15;			
	alt_3 = (101000 - pressure)/1000.0f;
	barometer.altitude = 0.82f *alt_3 * alt_3 *alt_3 + 0.09f *(101000 - pressure)*100.0f ;	
	barometer.altitude -= barometer.alt_offset;
	DerivativeFilter_Update(&barometer.climb_rate_filter, barometer.altitude, barometer.last_update);
	
	barometer.Temp = temperature*0.01f;

}

//样本求平均,求除温度和气压
uint8_t barometer_read()
{
	uint8_t updated = barometer.updated;
	if (updated)		//如果数据更新了（在定时器）
	{
		uint32_t sD1, sD2;
		uint8_t d1count, d2count;
		// Suspend timer procs because these variables are written to
		// in "_update".
		scheduler_suspend_timer_procs();		//停止采集
		sD1 = barometer.d1; barometer.d1 = 0;
		sD2 = barometer.d2; barometer.d2 = 0;
		d1count = barometer.d1_count; barometer.d1_count = 0;
		d2count = barometer.d2_count; barometer.d2_count = 0;
		barometer.updated = 0;
		scheduler_resume_timer_procs();

		if (d1count != 0) 
		{
			barometer.D1 = ((float)sD1) / d1count;
		}
		if (d2count != 0) 
		{
			barometer.D2 = ((float)sD2) / d2count;
		}
		barometer.pressure_samples = d1count;
		barometer.last_update = millis();
		barometer_calculate();
	}
	return updated ? 1 : 0;
}


//气压计校准,获取地面气压
void barometer_calibrate()
{
	float ground_pressure = 0;
	float ground_temperature = 0;
	float alt_offset = 0.0f;
	uint32_t tstart;
	uint32_t i;

	barometer.alt_offset = 0;
	barometer.ground_pressure = 0;

	//读取第一个数据
	tstart = millis();
	while (ground_pressure == 0)		//读取地面气压
	{
		barometer_read();		//读取温度，气压到Temp，Press	
		if (millis() - tstart > 500)		//超时？？
		{
		}
		ground_pressure = barometer.Press;
		ground_temperature = barometer.Temp;
		delay_ms(100);
	}

	//读取一段时间的气压？？丢弃
	for (i = 0; i < 10; i++)
	{
		tstart = millis();
		barometer_read();		//读取温度，气压到Temp，Press	
		if (millis() - tstart > 500)		//超时？？
		{

		}
		ground_pressure = barometer.Press;
		ground_temperature = barometer.Temp;
		delay_ms(100);
	}

	//读取地面气压，滤波
	for (i = 0; i < 5; i++)
	{
		tstart = millis();
		barometer_read();		//读取温度，气压到Temp，Press	
		if (millis() - tstart > 500)		//超时？？
		{
			
		}
		ground_pressure = ground_pressure*0.8 + barometer.Press*0.2;
		ground_temperature = ground_temperature*0.8 + barometer.Temp*0.2;
		alt_offset += 0.2*barometer.altitude;
		delay_ms(100);
	}
	barometer.alt_offset = alt_offset;
	barometer.altitude -= alt_offset;
	barometer.ground_pressure = ground_pressure;
	barometer.ground_temperature = ground_temperature;
}


//TODO:没有使用最新的温度???
float barometer_get_altitude_difference(float base_pressure, float pressure)
{
	// on faster CPUs use a more exact calculation
	float ret;
	float scaling = pressure / base_pressure;
	float temp = barometer.ground_temperature + 273.15f;//温度转换?

	// This is an exact calculation that is within +-2.5m of the standard atmosphere tables
	// in the troposphere (up to 11,000 m amsl).
	ret = 153.8462f * temp * (1.0f - expf(0.190259f * logf(scaling)));
	return ret;
}


float barometer_get_climbrate()
{
	return DerivativeFilter_Slope(&barometer.climb_rate_filter)*1000.0f;
}
float barometer_get_altitude()
{
//	float alt;
//	if (barometer.ground_pressure == 0)
//	{
//		return 0;
//	}
	
	
//	if (barometer.last_altitude_t == barometer.last_update)
//	{
//		return barometer.altitude - barometer.alt_offset;
//	}
//	
//	alt = barometer_get_altitude_difference(barometer.ground_pressure, barometer.Temp);
//	barometer.last_altitude_t = barometer.last_update;

//	if (isnan(alt) || isinf(alt))
//	{ 
//		barometer.flags.alt_ok = 0;
//	}
//	else
//	{
//		barometer.altitude = alt;
//		barometer.flags.alt_ok = 1;
//	}

// 	// ensure the climb rate filter is updated
// 	_climb_rate_filter.update(_altitude, _last_update);
	return barometer.altitude;// - barometer.alt_offset;
}




void barometer_update()
{
	uint32_t now;
	now = micros();
	if (now - barometer.start_time < 10000)//10ms
	{
		return;
	}
	
	if(!myiic_sem_take_nonblocking())return;
	barometer.sample_dt = now - barometer.start_time;
	barometer.start_time = now;
	if (barometer.state == 0)//读取温度
	{
		uint32_t d2 = ms5611_read_Adc_T();
		if (d2 != 0)
		{
			barometer.d2 += d2;
			barometer.d2_count++;
			if (barometer.d2_count == 32)
			{
				barometer.d2 >>= 1;
				barometer.d2_count = 16;
			}
		}
		barometer.state++;
		ms5611_start_P();
	}
	else
	{
		uint32_t d1 = ms5611_read_Adc_P();
		if (d1 != 0)
		{
			barometer.d1 += d1;
			barometer.d1_count++;
			if (barometer.d1_count == 128)
			{
				barometer.d1 >>= 1;
				barometer.d1_count = 64;
			}
			barometer.updated = 1;
		}
		barometer.state++;
		if (barometer.state == 5)
		{
			ms5611_start_T();
			barometer.state = 0;
		}
		else
		{
			ms5611_start_P();
		}
	}
	myiic_sem_give();
}
