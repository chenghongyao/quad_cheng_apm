#include "usart.h"
#include "sys.h"
#include "ano.h"
#include "csp.h"
#include "ultrasound.h"

#include "string.h"
#include "sbus.h"


void usart1_init()
{
	H_USART1_ENRCC();
	
	H_USART_Init(1,500000,USART_Mode_Rx | USART_Mode_Tx);
	H_USART_IT_RXNE(1,2,2);
	
	H_GPIO_Init(A,9,GPIO_Speed_50MHz,GPIO_Mode_AF_PP);
	H_GPIO_Init(A,10,GPIO_Speed_50MHz,GPIO_Mode_IN_FLOATING);

	
}

void usart2_init()
{
		/*USART2*/
	H_USARTx_ENRCC(2);
	//H_USART_Init(2,115200,USART_Mode_Rx | USART_Mode_Tx);
	
	{	//for sbus
		USART_InitTypeDef USART_InitStruct = {100000,USART_WordLength_9b,USART_StopBits_2,USART_Parity_Even,USART_Mode_Rx,USART_HardwareFlowControl_None};																	
		USART_Init(USART2,&USART_InitStruct);
		USART_Cmd(USART2,ENABLE);
	}
	
	H_USART_IT_RXNE(2,0,0);		//低优先级会出错
//	H_GPIO_Init(A,2,GPIO_Speed_50MHz,GPIO_Mode_AF_PP);			//TX
	H_GPIO_Init(A,3,GPIO_Speed_50MHz,GPIO_Mode_IN_FLOATING);	//RX
	


}


void usart3_init()
{
	H_USARTx_ENRCC(3);
	H_USART_Init(3,500000,USART_Mode_Rx | USART_Mode_Tx);
	H_USART_IT_RXNE(3,2,2);
	
//	H_GPIO_Init(B,10,GPIO_Speed_50MHz,GPIO_Mode_AF_PP);			//TX
	H_GPIO_Init(B,11,GPIO_Speed_50MHz,GPIO_Mode_IN_FLOATING);	//RX
}

void myputc(uint8_t c)
{
	H_USART_PUT8_F(1,c);
}


void myputbuf(uint8_t *buffer, uint16_t len)
{
	uint16_t i;
	for ( i = 0; i < len; i++)
	{
		myputc(*buffer++);
	}
}
void myputs(uint8_t *s)
{
	while(*s)
	{
		myputc(*s++);
	}
	myputc('\r');
	myputc('\n');
}


void usart3_putc(uint8_t c)
{
	H_USART_PUT8_F(3,c);
}
void usart3_puts(uint8_t *s)
{
	while(*s)
	{
		usart3_putc(*s++);
	}
	usart3_putc('\r');
	usart3_putc('\n');
}

void usart3_putbuf(uint8_t *buffer, uint16_t len)
{
	uint16_t i;
	for ( i = 0; i < len; i++)
	{
		usart3_putc(*buffer++);
	}
}

//////////////////////////////////////////////////////////////////////////
void USART1_IRQHandler(void)
{

	if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{
		uint8_t Res;
		Res = USART_ReceiveData(USART1);
		if(Res)
		{
		
		}
		//ultrasound_get(Res);
		
	}
}


void USART2_IRQHandler(void) //
{
	
	
	if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
	{
		uint8_t Res;
		uint32_t now;
		static uint32_t last_byte_time=0;
		
		Res = USART2->DR; 	

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
	}	
}



void USART3_IRQHandler(void) 
{
	if (USART_GetITStatus(USART3,USART_IT_RXNE) != RESET)
	{
		uint8_t Res;
		Res =USART3->DR; 
		if(Res)
		{
		
		}
		//csp_recvByte(Res);
	}
}



