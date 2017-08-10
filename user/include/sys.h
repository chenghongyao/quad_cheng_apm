#ifndef __SYS_H
#define __SYS_H	  

#include <stm32f10x.h>   
#include "stdint.h"
#include "lib_helper.h"
#include "flash.h"
#include "stdio.h"
#include "delay.h"
#include "table.h"

#include "myiic.h"
#include "nrf.h"
#include "esp8266.h"
#include "mpu6050.h"
#include "usart.h"
#include "motor.h"
#include "led.h"
#include "ultrasound.h"
#include "ms5611.h"
#include "hmc5883.h"


#include "bytes.h"
#include "wos.h"
#include "cycle.h"
#include "cmath.h"
#include "ano.h"

#include "config.h"

#define _DEBUG
#define _LOGT			//发送时间标签


#ifdef _DEBUG
	#define debug(fmt,...) 	printf(fmt,##__VA_ARGS__)
#else
	#define debug(fmt,...)		do{}while(0)
#endif
		
		
#ifdef _LOGT			//时间标签
		#define logt(msg) printf("%s:us",micros())
#else
	#define logt(msg,_us)		do{}while(0)
#endif
		
		
		
		
#define NVIC_GROUP	2		//NVIC分组
//Ex_NVIC_Config专用定义
#define GPIO_A 0
#define GPIO_B 1
#define GPIO_C 2
#define GPIO_D 3
#define GPIO_E 4
#define GPIO_F 5
#define GPIO_G 6 		
#define FTIR   1  //下降沿触发
#define RTIR   2  //上升沿触发		
		
		
		
		
//JTAG模式设置定义
#define JTAG_SWD_DISABLE   0X02
#define SWD_ENABLE         0X01
#define JTAG_SWD_ENABLE    0X00	







//外部中断
#define H_EXTI_ISPEND_F(_line)		(EXTI->PR&EXTI_PR_PR##_line)
#define H_EXTI_CLEARPEND_F(_line)	do{(EXTI->PR |= EXTI_PR_PR##_line);}while(0)






//board hardware
//LED
#define ledM PCout(13)
#define beepM	PAout(0)
#define LEDM_ON()			ledM = 0
#define LEDM_OFF()		ledM = 1


//NRF24L01
#define NRF_IRQ PAin(8)  
#define NRF_CSN PBout(12)
#define NRF_CE PAout(11)
#define NRF_ReadWriteByte(_c) H_SPI_RWByte(2,_c)

//MOTOR
#define MOTOR_TR (TIM3->CCR1)
#define MOTOR_TL (TIM3->CCR2)
#define MOTOR_HL (TIM3->CCR3)
#define MOTOR_HR (TIM3->CCR4)	



//ultrasound
#define ULTRA_TRIG	PBout(9)
#define ULTRA_ECHO	PBin(8)

#define ULTRA_PUTC(_c)	H_USART_PUT8_F(3,_c)

//IIC
#define IIC_SDA  PBout(5)            
#define IIC_SDAI PBin(5)
#define IIC_SCL PBout(4)

 #define IIC_SCL_OUT()	do{H_GPIO_SETMODEL_F(B,4,F_GPIO_Speed_2M,F_GPIO_Mode_Out_PP);}while(0)
 #define IIC_SDA_OUT()	do{H_GPIO_SETMODEL_F(B,5,F_GPIO_Speed_2M,F_GPIO_Mode_Out_PP);}while(0)
 #define IIC_SDA_IN()	do{H_GPIO_SETMODEL_F(B,5,F_GPIO_Speed_IN,F_GPIO_Mode_IPU);}while(0)



void Sys_Soft_Reset(void);      //系统软复位
void Sys_Standby(void);         //待机模式 	
void JTAG_Set(uint8_t mode);
void MSR_MSP(u32 addr);	//设置堆栈地址

void board_setup(void);


#endif














