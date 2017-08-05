#include "mpu6050_dev.h"
#include "mpu6050.h"
#include "sys.h"

//#include "cmath.h"





//底层驱动接口===============================================================
static void MPU_WriteByte(u8 reg_addr,u8 reg_dat)
{
	 IIC_DevWriteReg(MPU_SlaveAddress,reg_addr, reg_dat);
}
static void MPU_ReadBuf(u8 reg_addr,u8* pBuf,u8 len)
{
	IIC_DevReadBuf(MPU_SlaveAddress,reg_addr,len,pBuf);
}
//=================================================================



void mpu6050_init()
{
	MPU_WriteByte(PWR_MGMT_1, 0x80);						//复位
	delay_ms(50);
	
	//陀螺仪采样频率设置 采样样频率=陀螺仪输出频率/(1+SMPLRT_DIV),
	//DLPF使能时陀螺仪输出频率为1k,否则为8k
	//这一句设置采样不分频
	MPU_WriteByte(SMPLRT_DIV, 0x00);	//设置陀螺仪采样频率
	MPU_WriteByte(PWR_MGMT_1, 0x03);	//解除休眠状态,PLLwith gyroz
	
	MPU_WriteByte(PIN_CONFIG, 0x02);	//IIC从机管脚直通
	MPU_WriteByte(USER_CTRL, 0x00);
	
	MPU_WriteByte(MPU_CONFIG, MPU6050_DLPF_BW_20);	///DLPF设置:ACC=44Hz,GYRO=42hZ(输出频率1k)
	MPU_WriteByte(GYRO_CONFIG, MPU_GYROCFG_FSSEL_2000);	//设置量程
	MPU_WriteByte(ACCEL_CONFIG,MPU_ACCELCFG_AFSSEL_8G);
	
	delay_ms(100);	//等待模块稳定,如果立即读发现数据偏差太大
}





void mpu6050_getGyroAdc(int16_t *gyroadc)
{
	uint8_t  gyrobuffer[6];
	MPU_ReadBuf(GYRO_XOUT_H,gyrobuffer,6);		
	gyroadc[0]=(int16_t)((gyrobuffer[0]<<8) | gyrobuffer[1]);
	gyroadc[1]=(int16_t)((gyrobuffer[2]<<8) | gyrobuffer[3]);
	gyroadc[2]=(int16_t)((gyrobuffer[4]<<8) | gyrobuffer[5]);
}

void mpu6050_getAccAdc(int16_t *accadc)
{
	uint8_t accbuffer[6];
	MPU_ReadBuf(ACCEL_XOUT_H,accbuffer,6);		
	accadc[0]=(int16_t)((accbuffer[0]<<8) | accbuffer[1]);
	accadc[1]=(int16_t)((accbuffer[2]<<8) | accbuffer[3]);
	accadc[2]=(int16_t)((accbuffer[4]<<8) | accbuffer[5]);
}


void mpu6050_getAccel(float *acc)
{
	uint8_t accbuffer[6];
	MPU_ReadBuf(ACCEL_XOUT_H,accbuffer,6);		
	acc[0]=MPU_K_ACCEL*(int16_t)((accbuffer[0]<<8) | accbuffer[1]);
	acc[1]=MPU_K_ACCEL*(int16_t)((accbuffer[2]<<8) | accbuffer[3]);
	acc[2]=MPU_K_ACCEL*(int16_t)((accbuffer[4]<<8) | accbuffer[5]);
}


/////////////////////////////////////////////////////
//单位:度/s
////////////////////////////////////////////////////
void mpu6050_getGyro(float *gyro)
{
	uint8_t  gyrobuffer[6];
	MPU_ReadBuf(GYRO_XOUT_H,gyrobuffer,6);		
	gyro[0] = MPU_K_GYRO*(int16_t)((gyrobuffer[0] << 8) | gyrobuffer[1]);
	gyro[1] = MPU_K_GYRO*(int16_t)((gyrobuffer[2] << 8) | gyrobuffer[3]);
	gyro[2] = MPU_K_GYRO*(int16_t)((gyrobuffer[4] << 8) | gyrobuffer[5]);
}
	
