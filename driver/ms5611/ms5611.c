#include "ms5611.h"
#include "ms5611_dev.h"
#include "sys.h"


#include "alt.h"
#include "filter.h"

static uint16_t  ms5611_prom[8]; //标定值存放
static uint32_t adc_T;
static uint32_t adc_P;

static 	s32 baro_Offset;
float ms5611_temperature;

#define MO_LEN 5
s32 mo_av_baro[MO_LEN];
u16 moavcnt;

// 延时表单位 us不同的采样精度对应不同的延时值
//static  const uint32_t  MS5611_Delay_us[9] = {
//	1500,	//MS561101BA_OSR_256 0.9ms  0x00
//	1500,	//MS561101BA_OSR_256 0.9ms  
//	2000,	//MS561101BA_OSR_512 1.2ms  0x02
//	2000,	//MS561101BA_OSR_512 1.2ms
//	3000,	//MS561101BA_OSR_1024 2.3ms 0x04
//	3000,	//MS561101BA_OSR_1024 2.3ms
//	5000,	//MS561101BA_OSR_2048 4.6ms 0x06
//	5000,	//MS561101BA_OSR_2048 4.6ms
//	11000,//MS561101BA_OSR_4096 9.1ms 0x08		//10ms够了
//};


void ms5611_reset()
{	
	IIC_DevWriteByte(MS5611BA_ADDR,MS561101BA_RESET);
}
void ms5611_readPROM(void)
{
	uint8_t  i;
	uint8_t buf[2];
	for (i=0;i<8;i++) 
	{	
		IIC_DevReadBuf(MS5611BA_ADDR,MS561101BA_PROM_BASE_ADDR + i*2,2,buf);	
		ms5611_prom[i] = ((buf[0] << 8) | buf[1]);
	}
}
void ms5611_start_T()
{
	IIC_DevWriteByte(MS5611BA_ADDR,MS561101BA_D2+MS5611Temp_OSR);
}
void ms5611_start_P()
{
	IIC_DevWriteByte(MS5611BA_ADDR,MS561101BA_D1+MS5611Press_OSR);
}

void ms5611_read_Adc_T()
{
	uint8_t buf[3];
	IIC_DevReadBuf(MS5611BA_ADDR,MS5611BA_READ,3,buf);
	adc_T = (buf[0] << 16) | (buf[1] << 8) | buf[2];//温度
}
void ms5611_read_Adc_P()
{
	uint8_t buf[3];
	IIC_DevReadBuf(MS5611BA_ADDR,MS5611BA_READ,3,buf);
	adc_P = (buf[0] << 16) | (buf[1] << 8) | buf[2];//温度
}

void ms5611_getBaroAlt()
{
	
	static uint8_t baro_start = 0;
	
	int32_t off2 = 0, sens2 = 0, delt;
	int32_t temperature,pressure;
	int32_t dT;
	int64_t off;
	int64_t sens;
	float alt_3;
	dT = adc_T - ((uint32_t)ms5611_prom[5] << 8);		//温度差
	off = ((uint32_t)ms5611_prom[2] << 16) + (((int64_t)dT * ms5611_prom[4]) >> 7);
	sens = ((uint32_t)ms5611_prom[1] << 15) + (((int64_t)dT * ms5611_prom[3]) >> 8);
	
	temperature = 2000 + (((int64_t)dT * ms5611_prom[6]) >> 23);					//温度值*100度

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
	pressure = (((adc_P * sens ) >> 21) - off) >> 15;			//气压
	alt_3 = (101000 - pressure)/1000.0f;
	pressure = 0.82f *alt_3 * alt_3 *alt_3 + 0.09f *(101000 - pressure)*100.0f ;		//气压->高度
	
	
	sensor_baro.height =  Moving_Median(mo_av_baro,MO_LEN ,&moavcnt,(s32)(pressure - baro_Offset));//中值滤波后的高度
	sensor_baro.height_delta = sensor_baro.height - sensor_baro.height_old;
	sensor_baro.height_old = sensor_baro.height;
	sensor_baro.mesure_status = 1;
	
	//矫正
	if(baro_start < 60)
	{
		baro_start ++;
		sensor_baro.height_delta = 0;
		sensor_baro.height = 0;
		if(baro_start<10)
		{
			baro_Offset = pressure;		//预读取10次
		}
		else
		{
			baro_Offset +=  FILTER_EASY_COEF(5,0.02)*(pressure - baro_Offset);	//气压低通滤波
		}
	}
	
	
	ms5611_temperature += 0.01f *( ( 0.01f *temperature ) - ms5611_temperature );
		
}





//更新周期越20Hz
uint8_t ms5611_update()
{
	static uint8_t  step = 1;
	if(step)	//读取温度，启动气压转换
	{
		ms5611_read_Adc_T();		//读取温度
		ms5611_start_P();				//启动气压转换
		step = 0;
	}
	else		//读取气压，启动温度转换
	{
		ms5611_read_Adc_P();	//读取气压
		ms5611_start_T();			//启动温度转换
		ms5611_getBaroAlt();	//获取气压高度
		step = 1;
	}
	
	return step;
}


void ms5611_init()
{
	ms5611_reset();
	delay_ms(10);
	ms5611_readPROM();
	delay_ms(100);
	ms5611_start_T();		//启动温度测量
	
	

}

