#ifndef _HMC5883_H
#define _HMC5883_H
#include "sys.h"







#define HMC5883_COUNTS_GA	1370.0f		//X/高斯
#define HMC5883_KMAG			1.0f/HMC5883_COUNTS_GA




void hmc5883_init(void);
void hmc5883_getMagAdc(int16_t *mag);
void hmc5883_getMag(float *mag);
void hmc5883_getID(uint8_t *id);

#endif
