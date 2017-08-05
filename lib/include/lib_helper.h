#ifndef _FWLIB_HELPER_H
#define _FWLIB_HELPER_H

#include "sys.h"

/*
H_ 	库实现
HF_ 寄存器实现
HW_	软仿

*/

/*RCC*/
//时钟打开
#define H_RCC_ENAHB(_peripheral) 	do{RCC_AHBPeriphClockCmd(RCC_AHBPeriph_##_peripheral,ENABLE);}while(0)
#define H_RCC_ENAPB1(_peripheral) 	do{RCC_APB1PeriphClockCmd(RCC_APB1Periph_##_peripheral,ENABLE);}while(0)
#define H_RCC_ENAPB2(_peripheral) 	do{RCC_APB2PeriphClockCmd(RCC_APB2Periph_##_peripheral,ENABLE);}while(0)

//时钟关闭
#define H_RCC_STOPAHB(_peripheral) 	do{RCC_AHBPeriphClockCmd(RCC_AHBPeriph_##_peripheral,DISABLE);}while(0)
#define H_RCC_STOPAPB1(_peripheral) do{RCC_APB1PeriphClockCmd(RCC_APB1Periph_##_peripheral,DISABLE);}while(0)
#define H_RCC_STOPAPB2(_peripheral) do{RCC_APB2PeriphClockCmd(RCC_APB2Periph_##_peripheral,DISABLE);}while(0)

//时钟复位
#define H_RCC_RSTPAPB1(_peripheral) 	do{RCC_APB1PeriphResetCmd(RCC_APB1Periph_##_peripheral,DISABLE);}while(0)
#define H_RCC_RSTPAPB2(_peripheral) 	do{RCC_APB2PeriphResetCmd(RCC_APB2Periph_##_peripheral,DISABLE);}while(0)
/*RCC END*/

/*NVIC*/
#define H_NVIC_INIT(_perip,_pre,_sub) do{NVIC_InitTypeDef NVIC_InitStruct = {_perip##_IRQn,_pre,_sub,ENABLE}; NVIC_Init(&NVIC_InitStruct);}while(0)
#define H_NVIC_CLOSE(_perip) do{NVIC_InitTypeDef NVIC_InitStruct ={##_perip_IRQn,0,0,DISABLE}; NVIC_Init(&NVIC_InitStruct);}while(0)


/*NVIC END*/


/*EXTI*/
#define H_EXTI_Init(_port,_pin,_trigger)	do{EXTI_InitTypeDef EXTI_InitStructure={EXTI_Line##_pin,EXTI_Mode_Interrupt,_trigger,ENABLE};\
												H_RCC_ENAPB2(AFIO);	\
												GPIO_EXTILineConfig(GPIO_PortSourceGPIO##_port,GPIO_PinSource##_pin);\
												EXTI_Init(&EXTI_InitStructure);		}while(0)

#define H_EXTI_IT_ENABLE(_pin,_pre,_sub)	H_NVIC_INIT(EXTI##_pin,_pre,_sub)
/*EXTI END*/



/*GPIO*/
//IO口操作宏定义
#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2)) 
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr)) 
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum)) 
//IO口地址映射
#define GPIOA_ODR_Addr    (GPIOA_BASE+12) //0x4001080C 
#define GPIOB_ODR_Addr    (GPIOB_BASE+12) //0x40010C0C 
#define GPIOC_ODR_Addr    (GPIOC_BASE+12) //0x4001100C 
#define GPIOD_ODR_Addr    (GPIOD_BASE+12) //0x4001140C 
#define GPIOE_ODR_Addr    (GPIOE_BASE+12) //0x4001180C 
#define GPIOF_ODR_Addr    (GPIOF_BASE+12) //0x40011A0C    
#define GPIOG_ODR_Addr    (GPIOG_BASE+12) //0x40011E0C    

#define GPIOA_IDR_Addr    (GPIOA_BASE+8) //0x40010808 
#define GPIOB_IDR_Addr    (GPIOB_BASE+8) //0x40010C08 
#define GPIOC_IDR_Addr    (GPIOC_BASE+8) //0x40011008 
#define GPIOD_IDR_Addr    (GPIOD_BASE+8) //0x40011408 
#define GPIOE_IDR_Addr    (GPIOE_BASE+8) //0x40011808 
#define GPIOF_IDR_Addr    (GPIOF_BASE+8) //0x40011A08 
#define GPIOG_IDR_Addr    (GPIOG_BASE+8) //0x40011E08 
 
//IO口操作,只对单一的IO口!


//确保n的值小于16!
#define PAout(n)   BIT_ADDR(GPIOA_ODR_Addr,n)  //输出 
#define PAin(n)    BIT_ADDR(GPIOA_IDR_Addr,n)  //输入 

#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)  //输出 
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)  //输入 

#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr,n)  //输出 
#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr,n)  //输入 

#define PDout(n)   BIT_ADDR(GPIOD_ODR_Addr,n)  //输出 
#define PDin(n)    BIT_ADDR(GPIOD_IDR_Addr,n)  //输入 

#define PEout(n)   BIT_ADDR(GPIOE_ODR_Addr,n)  //输出 
#define PEin(n)    BIT_ADDR(GPIOE_IDR_Addr,n)  //输入

#define PFout(n)   BIT_ADDR(GPIOF_ODR_Addr,n)  //输出 
#define PFin(n)    BIT_ADDR(GPIOF_IDR_Addr,n)  //输入

#define PGout(n)   BIT_ADDR(GPIOG_ODR_Addr,n)  //输出 
#define PGin(n)    BIT_ADDR(GPIOG_IDR_Addr,n)  //输入


//IO口配置宏
#define F_GPIO_Mode_AIN							0x00			//模拟输入
#define F_GPIO_Mode_IN_FLOATING					0x04			//浮空输入
#define F_GPIO_Mode_IPD							0x08			//下拉输入
#define F_GPIO_Mode_IPU 							0x08			//上拉输入 通过设置ODR
#define F_GPIO_Mode_Out_PP 						0x00			//推挽输出
#define F_GPIO_Mode_Out_OD 						0x04			//开漏输出
#define F_GPIO_Mode_AF_PP 							0x08			//复用推挽输出
#define F_GPIO_Mode_AF_OD 							0x0C			//复用开漏输出

#define F_GPIO_Speed_IN							0x00			//输入模式
#define F_GPIO_Speed_10M							0x01			//10M
#define F_GPIO_Speed_2M							0x02			//2M
#define F_GPIO_Speed_50M							0x03			//50M


#define H_GPIO_SETMODEH_F(_port,_pin,_speed,_mode)		do{GPIO##_port->CRH&=~((u32)0x0F<<((_pin-8)*4));GPIO##_port->CRH|=(u32)(_mode|_speed)<<((_pin-8)*4);}while(0)
#define H_GPIO_SETMODEL_F(_port,_pin,_speed,_mode)		do{GPIO##_port->CRL&=~((u32)0x0F<<(_pin<<2));GPIO##_port->CRL|=(u32)(_mode|_speed)<<(_pin<<2);}while(0)
#define H_GPIO_SETIPU_F(_port,_pin) do{(GPIO##_port->ODR|=GPIO_ODR_ODR##_pin);}while(0)



#define H_GPIO_ENRCC(_port)									 H_RCC_ENAPB2(GPIO##_port)
#define H_GPIO_Init(_port,_pin,_speed,_mode) do{GPIO_InitTypeDef GPIO_InitStruct = {GPIO_Pin_##_pin,_speed,_mode};H_GPIO_ENRCC(_port);GPIO_Init(GPIO##_port,&GPIO_InitStruct);}while(0) 
/*GPIO END*/




/*USART*/


//8为数据位,1位停止为,无校验,无流控制
#define H_USART_Init(_channel,_baud,_mode)  do{USART_InitTypeDef USART_InitStruct = {_baud,USART_WordLength_8b,USART_StopBits_1,USART_Parity_No,_mode,USART_HardwareFlowControl_None};\
																					USART_Init(USART##_channel,&USART_InitStruct);\
																					USART_Cmd(USART##_channel,ENABLE);}while(0) 	

#define H_USART1_ENRCC()					H_RCC_ENAPB2(USART1)
#define H_USARTx_ENRCC(_channel)	H_RCC_ENAPB1(USART##_channel)	//不能是通道1
#define H_USART_IT_RXNE(_channel,_pre,_sub) do{H_NVIC_INIT(USART##_channel,_pre,_sub);USART_ITConfig(USART##_channel,USART_IT_RXNE,ENABLE);}while(0)
#define H_USART_PUT8_F(_channel,_c)	do{while(!(USART##_channel->SR&USART_SR_TXE));USART##_channel->DR = _c;}while(0)
/*USART END*/

/*SPI*/

//双线双向全双工,主SPI,8位,软件片选信号,crc:7??
#define H_SPI_Init(_channel,_cpol,_cpha,_firstbit,_div) do{SPI_InitTypeDef SPI_InitStruct = {SPI_Direction_2Lines_FullDuplex,SPI_Mode_Master,SPI_DataSize_8b,_cpol,_cpha,SPI_NSS_Soft,SPI_BaudRatePrescaler_##_div,_firstbit,7};\
																														SPI_Init(SPI##_channel,&SPI_InitStruct);\
																														SPI_Cmd(SPI##_channel, ENABLE);}while(0)

#define H_SPI_BaudRateDiv_F(_channel,_div)		do{SPI_Cmd(SPI##_channel, DISABLE);	\
																									SPI##_channel->CR1&=0XFFC7;SPI##_channel->CR1|=SPI_BaudRatePrescaler_##_div; \
																									SPI_Cmd(SPI##_channel, ENABLE);}while(0)																												

#define H_SPI1_ENRCC()					H_RCC_ENAPB2(SPI1)
#define H_SPIx_ENRCC(_channel)	H_RCC_ENAPB1(SPI##_channel)	//不能是通道1
						
extern uint8_t SPI_RWByte(SPI_TypeDef *SPIx,uint8_t x);																												
#define H_SPI_RWByte(_channel,_x)				SPI_RWByte(SPI##_channel,_x)
																									
/*SPI END*/
																									
																									
/*TIMEBASE*/
/*
																									
	H_TIMx_ENRCC(3);
	H_TIMEBASE_Init(3,72,1000);	//1ms定时器
	H_TIMBASE_IT_UPDATE(3,0,2);
	H_TIMEBASE_START(3);
																									
*/
																		
//向上计数	频率 72M/div/_period																						
#define  H_TIMEBASE_Init(_channel,_div,_period) do{TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct = {_div-1,TIM_CounterMode_Up,_period-1,TIM_CKD_DIV1,0};\
																									TIM_TimeBaseInit(TIM##_channel,&TIM_TimeBaseInitStruct);}while(0) 

#define H_TIMBASE_IT_UPDATE(_channel,_pre,_sub)	do{H_NVIC_INIT(TIM##_channel,_pre,_sub);TIM_ITConfig(TIM##_channel,TIM_IT_Update,ENABLE);}while(0)
#define H_TIMBASE_IT_UPDATE_CLOSE(_channel) do{TIM_ITConfig(TIM##_channel,TIM_IT_Update,DISABLE);}while(0)																	

#define H_TIMEBASE18_IT_UPDATE(_channel,_pre,_sub)	do{H_NVIC_INIT(TIM##_channel##_UP,_pre,_sub);TIM_ITConfig(TIM##_channel,TIM_IT_Update,ENABLE);}while(0)


#define  H_TIMEBASE_START(_channel) do{TIM_Cmd(TIM##_channel,ENABLE);}while(0)
#define  H_TIMEBASE_STOP(_channel)	do{TIM_Cmd(TIM##_channel,DISABLE);}while(0)
#define  H_TIMEBASE_RESTART_F2(_channel) do{H_TIMEBASE_STOP(_channel);	TIM##_channel->CNT = 0;H_TIMEBASE_START(_channel);}while(0)

#define  H_TIMEBASE_START_F(_channel) do{TIM_Cmd(TIM##_channel,ENABLE);}while(0)
#define  H_TIMEBASE_STOP_F(channel)	do{TIM##_channel->CR2 &= ~TIM_CR1_CEN;}while(0)
#define  H_TIMEBASE_RESTART_F(_channel) do{TIM##_channel->CR2 &= ~TIM_CR1_CEN;	TIM##_channel->CNT = 0;TIM##_channel->CR1 |= TIM_CR1_CEN;}while(0)

	
#define H_TIMEBASE_ISUPDATE_F(_channel) (TIM##_channel->SR&TIM_SR_UIF)			//读取标志位
#define H_TIMEBASE_CLEARUPDATE_F(_channel) (TIM##_channel->SR&=~TIM_SR_UIF)

#define H_TIM18_ENRCC(_channel)	H_RCC_ENAPB2(TIM##_channel)	//通道1,8					
#define H_TIMx_ENRCC(_channel)	H_RCC_ENAPB1(TIM##_channel)	//不能是通道1,8																								

/*TIMEBASE END*/

/*PWM*/
/*
//步进电机驱动 周期20ms,脉宽0-2.5ms(0.5-2.5ms)
	H_TIMx_ENRCC(3);
	H_TIMEBASE_Init(3,72,20000);
	H_GPIO_Init(A,6,GPIO_Speed_50MHz,GPIO_Mode_AF_PP);
	H_PWM_Init(3,1);
	H_TIMEBASE_START(3);

*/

//小于比较值输出高电平
#define H_PWM_Init(_tmchannel,_och) do{TIM_OCInitTypeDef TIM_OCInitStructure={TIM_OCMode_PWM1,TIM_OutputState_Enable,TIM_OutputNState_Disable,0,TIM_OCPolarity_High,TIM_OCNPolarity_High,TIM_OCIdleState_Reset,TIM_OCNIdleState_Reset};\
																				TIM_OC##_och##Init(TIM##_tmchannel, &TIM_OCInitStructure);\
																				TIM_OC##_och##PreloadConfig(TIM##_tmchannel, TIM_OCPreload_Enable);}while(0)    
#define H_PWM_SetCCRx(_tmchannel,_och,_val) do{TIM##_tmchannel->CCR##_och = (_val);}while(0);
/*PWM END*/


/*CAPTURE */
#define H_CAPTURE_Init(_tmchannel,_och) do{TIM_OCInitTypeDef TIM_OCInitStructure={TIM_OCMode_PWM1,TIM_OutputState_Enable,TIM_OutputNState_Disable,0,TIM_OCPolarity_High,TIM_OCNPolarity_High,TIM_OCIdleState_Reset,TIM_OCNIdleState_Reset};\
																				TIM_OC##_och##Init(TIM##_tmchannel, &TIM_OCInitStructure);\
																				TIM_OC##_och##PreloadConfig(TIM##_tmchannel, TIM_OCPreload_Enable);}while(0)    



/*CAPTURE END*/



#endif
