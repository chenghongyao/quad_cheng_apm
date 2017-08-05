#ifndef _MPU6050_DEV_H
#define _MPU6050_DEV_H


//GYRO_CONFIG
#define MPU_GYROCFG_FSSEL_250		(0<<3)		
#define MPU_GYROCFG_FSSEL_500		(1<<3)	
#define MPU_GYROCFG_FSSEL_1000		(2<<3)
#define MPU_GYROCFG_FSSEL_2000		(3<<3)

//ACCEL_CONFIG
#define MPU_ACCELCFG_AFSSEL_2G		(0<<3)		
#define MPU_ACCELCFG_AFSSEL_4G		(1<<3)	
#define MPU_ACCELCFG_AFSSEL_8G		(2<<3)
#define MPU_ACCELCFG_AFSSEL_16G		(3<<3)

//CONFIG DPF
#define MPU6050_DLPF_BW_256         0x00
#define MPU6050_DLPF_BW_188         0x01
#define MPU6050_DLPF_BW_98          0x02
#define MPU6050_DLPF_BW_42          0x03
#define MPU6050_DLPF_BW_20          0x04
#define MPU6050_DLPF_BW_10          0x05
#define MPU6050_DLPF_BW_5           0x06




/*定义MPU6050内部地址*/
#define	SMPLRT_DIV		0x19	//陀螺仪采样率，典型值：0x07(125Hz)
#define	MPU_CONFIG		0x1A	//低通滤波频率，典型值：0x06(5Hz)
#define	GYRO_CONFIG		0x1B	//陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)
#define	ACCEL_CONFIG	0x1C	//加速计自检、测量范围及高通滤波频率，典型值：0x01(不自检，2G，5Hz)
#define PIN_CONFIG		0x37
#define USER_CTRL     0x6A  

//三轴加速度
#define	ACCEL_XOUT_H	0x3B
#define	ACCEL_XOUT_L	0x3C
#define	ACCEL_YOUT_H	0x3D
#define	ACCEL_YOUT_L	0x3E
#define	ACCEL_ZOUT_H	0x3F
#define	ACCEL_ZOUT_L	0x40
//温度
#define	TEMP_OUT_H		0x41
#define	TEMP_OUT_L		0x42
//三轴角速度
#define	GYRO_XOUT_H		0x43
#define	GYRO_XOUT_L		0x44	
#define	GYRO_YOUT_H		0x45
#define	GYRO_YOUT_L		0x46
#define	GYRO_ZOUT_H		0x47
#define	GYRO_ZOUT_L		0x48

#define	PWR_MGMT_1					0x6B			//电源管理，典型值：0x00(正常启用)
#define	WHO_AM_I						0x75			//IIC地址寄存器(默认数值0x68，只读)
#define	MPU_SlaveAddress		0xD0			//IIC写入时的地址字节数据
//===============================================================================


#endif
