#ifndef _MS5611_H
#define _MS5611_H
#include "sys.h"




#define MS5611Press_OSR  MS561101BA_OSR_4096  //气压采样精度
#define MS5611Temp_OSR   MS561101BA_OSR_4096  //温度采样精度

#define MS5611_PREINIT_NUM 	300
#define MS5611_INIT_NUM 		100


typedef struct
{
	uint8_t IsBaroUpdated;
	
	float Temperature;			//温度 度
	float Pressure;					//压力 帕
	float PressureZero;			//0m气压值
	float Altitude;					//高度 m
	float alt_raw;
	float alt_offset;
	uint8_t PressInited;
}ms5611_t;
extern ms5611_t mMS5611;

void ms5611_init(void);
uint8_t ms5611_update(void);
float ms5611_getAltitude(void);





#endif

