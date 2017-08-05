#ifndef _MS5611_DEV_H
#define _MS5611_DEV_H
#include "sys.h"



#define MS5611BA_ADDR 0xEE

// registers of the device
#define MS561101BA_D1 		0x40          //气压
#define MS561101BA_D2 		0x50          //温度
#define MS561101BA_RESET 0x1E				//复位
#define MS5611BA_READ			0x00

#define MS561101BA_PROM_BASE_ADDR 0xA0 // by adding ints from 0 to 6 we can read all the prom configuration values. 



// OSR (Over Sampling Ratio) constants
#define MS561101BA_OSR_256  0x00  //Conversion time 0.6ms  Resolution 0.065mbar
#define MS561101BA_OSR_512  0x02  //Conversion time 1.2ms  Resolution 0.042mbar
#define MS561101BA_OSR_1024 0x04 //Conversion time 2.3ms  Resolution 0.027mbar
#define MS561101BA_OSR_2048 0x06 //Conversion time 4.6ms  Resolution 0.018mbar
#define MS561101BA_OSR_4096 0x08 //Conversion time 9.1ms  Resolution 0.012mbar


#endif
