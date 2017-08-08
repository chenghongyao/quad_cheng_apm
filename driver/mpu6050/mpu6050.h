#ifndef MPU6050_H
#define MPU6050_H
#include "sys.h"



#define CONSTANTS_ONE_G					9.80665f	
//根据初始化值对该量程进行修改
#define MPU6050_MAXG	8.0f				//重力量程 
#define MPU6050_MAXW	2000.0f			//角速度量程

//角速度与加速度ADC比例值,真实值为ADC值乘以该比例值
#define MPU_K_GYRO	(MPU6050_MAXW/32768.0f)
#define MPU_K_ACCEL	(MPU6050_MAXG*CONSTANTS_ONE_G/32768.0f)
//=======================================================================

void mpu6050_init(void);			//MPU初始化
uint8_t mpu6050_getGyroAdc(int16_t *gyroadc);
uint8_t mpu6050_getAccAdc(int16_t *accadc);
void mpu6050_getAccel(float *acc);
void mpu6050_getGyro(float *gyro);
uint8_t mpu6050_getAdcAll(int16_t *accadc,int16_t *gyroadc);
#endif

