#ifndef _SBUS_H_
#define _SBUS_H_

#include "sys.h"


/*
SBUS
一帧数据25字节,
每个通道11位,有16个通道,共22字节

futab实测:
中位值=1024
最小值=352
最大值1696

行程=1344(672+672)	
*/



#define SBUS_MID				1024
#define SBUS_RANG				1344
#define SBUS_MIN				(uint16_t)(SBUS_MID - SBUS_RANG/2)	
#define SBUS_MAX				(uint16_t)(SBUS_MID + SBUS_RANG/2)	


#pragma pack(1) 
//位域,先定义在高位
typedef	struct
{
	uint8_t startbyte;//0x0f
	uint32_t ch1:11;	//副翼,ALI/ROLL
	uint32_t ch2:11;	//升降,ELE/PITCH
	uint32_t ch3:11;	//油门,THR
	uint32_t ch4:11;	//尾翼,RUD/YAW

	uint32_t ch5:11;
	uint32_t ch6:11;
	uint32_t ch7:11;
	uint32_t ch8:11;
	uint32_t ch9:11;
	uint32_t ch10:11;
	uint32_t ch11:11;
	uint32_t ch12:11;
	uint32_t ch13:11;
	uint32_t ch14:11;
	uint32_t ch15:11;
	uint32_t ch16:11;	

	uint8_t dg1:1;				//digital channel (0x80)
	uint8_t dg2:1;				//digital channel (0x40)
	uint8_t framelost:1;	// Frame lost, equivalent red LED on receiver (0x20)
	uint8_t failsafe:1;		//failsafe activated (0x10)
	uint8_t reserve:4;
	uint8_t endbyte;			//0x00
}sbus_channel_t;
#pragma pack() 

typedef struct 
{
	uint32_t last_update;
	uint16_t rx_cnt;
	
	uint8_t buffer[25];				//startbyte,data1,data2,......,data22,flags,endbyte
	sbus_channel_t channel;
}sbus_t;

extern sbus_t sbus;



/*
////////////////////////////////////////////////////
//串口初始化
		{
			USART_InitTypeDef USART_InitStruct = {100000,USART_WordLength_9b,USART_StopBits_2,USART_Parity_Even,USART_Mode_Rx | USART_Mode_Tx,USART_HardwareFlowControl_None};																	
			USART_Init(USART1,&USART_InitStruct);
			USART_Cmd(USART1,ENABLE);
		}
		
/////////////////////////////////////////////////////////////////////
//接受处理
		uint32_t now;
		static uint32_t last_byte_time=0;

		now = micros();
		if((now - last_byte_time) > 3000)		//间隔大于3ms,新的一帧
		{
			if((sbus.rx_cnt==25) && (sbus.buffer[24]==0x00))		//上一帧校验正确
			{
				memcpy(&sbus.channel,sbus.buffer,25);	
				sbus.last_update = now;				
			}	
			
			if((sbus.buffer[0] = Res) == 0x0F)		//这一帧起始字节判断
			{
				sbus.rx_cnt = 1;
			}
		}
		else if(sbus.rx_cnt <25)
		{
			sbus.buffer[sbus.rx_cnt++] = Res;
		}
		last_byte_time = now;	
		
//////////////////////////////////////////////////////////////
	if(sbus_last_update != sbus.last_update)
	{

		ANO_DT_Send_MotoPWM(sbus.channel.ch1,sbus.channel.ch2,sbus.channel.ch3,sbus.channel.ch4,sbus.channel.ch5,sbus.channel.ch6,sbus.channel.ch7,sbus.channel.ch8);
		sbus_last_update = sbus.last_update;
	}
//////////////////////////////////////////////////////////////
*/
void sbus_init(void);
#endif

