#include "sys.h" 
#include "usart.h"
#include "myiic.h"
#include "Scheduler.h"


__asm void MSR_MSP(u32 addr) 
{
    MSR MSP, r0 			//set Main Stack value
    BX r14
}
//进入待机模式	  
void Sys_Standby(void)
{
	SCB->SCR|=1<<2;//使能SLEEPDEEP位 (SYS->CTRL)	   
  	RCC->APB1ENR|=1<<28;     //使能电源时钟	    
 	PWR->CSR|=1<<8;          //设置WKUP用于唤醒
	PWR->CR|=1<<2;           //清除Wake-up 标志
	PWR->CR|=1<<1;           //PDDS置位		
	__wfi();	 
}	     
//系统软复位   
void Sys_Soft_Reset(void)
{   
	SCB->AIRCR =0X05FA0000|(uint32_t)0x04;	  
} 		 
//JTAG模式设置,用于设置JTAG的模式
//mode:jtag,swd模式设置;00,全使能;01,使能SWD;10,全关闭;	   
//#define JTAG_SWD_DISABLE   0X02
//#define SWD_ENABLE         0X01
//#define JTAG_SWD_ENABLE    0X00		  
void JTAG_Set(uint8_t mode)
{
	uint32_t temp;
	temp=mode;
	temp<<=25;
	H_RCC_ENAPB2(AFIO);		//使能复用IO时钟	   
	AFIO->MAPR&=0XF8FFFFFF; //清除MAPR的[26:24]
	AFIO->MAPR|=temp;       //设置jtag模式
}





//中断,引脚,外设,系统模块
void board_setup(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_SetVectorTable(NVIC_VectTab_FLASH,0);
	JTAG_Set(SWD_ENABLE);
	delay_init();					//延时函数一定最先初始化
	
	//*LED*/
	H_GPIO_Init(C,13,GPIO_Speed_50MHz,GPIO_Mode_Out_PP); //LED
	H_GPIO_Init(A,0,GPIO_Speed_50MHz,GPIO_Mode_Out_PP); //BEEP
	beepM = 0;
	
	
		/*TIM3 PWM*/
	H_TIMx_ENRCC(3);
	H_TIMEBASE_Init(3,72,2500);
	H_PWM_Init(3,1);
	H_PWM_Init(3,2);
	H_PWM_Init(3,3);
	H_PWM_Init(3,4);

	H_GPIO_Init(B,0,GPIO_Speed_50MHz,GPIO_Mode_AF_PP);
	H_GPIO_Init(B,1,GPIO_Speed_50MHz,GPIO_Mode_AF_PP);
	H_GPIO_Init(A,6,GPIO_Speed_50MHz,GPIO_Mode_AF_PP);
	H_GPIO_Init(A,7,GPIO_Speed_50MHz,GPIO_Mode_AF_PP);

	H_PWM_SetCCRx(3,1,0);
	H_PWM_SetCCRx(3,2,0);
	H_PWM_SetCCRx(3,3,0);
	H_PWM_SetCCRx(3,4,0);
	
	H_TIMEBASE_START(3);
	
	//NRF24L01 
	H_GPIO_Init(A,8,GPIO_Speed_50MHz,GPIO_Mode_IPU); 	//IRQ
	H_GPIO_Init(B,12,GPIO_Speed_50MHz,GPIO_Mode_Out_PP);//CSN
	H_GPIO_Init(A,11,GPIO_Speed_50MHz,GPIO_Mode_Out_PP); //CE
	NRF_CE = 0;
	NRF_CSN = 1;
	
	/*SPI2 NRF24L01*/
	H_GPIO_Init(B,15,GPIO_Speed_50MHz,GPIO_Mode_AF_PP);//MOSI
	H_GPIO_Init(B,14,GPIO_Speed_50MHz,GPIO_Mode_AF_PP);//MISO
	H_GPIO_Init(B,13,GPIO_Speed_50MHz,GPIO_Mode_AF_PP);//SCK
	
	H_SPIx_ENRCC(2);
	H_SPI_Init(2,SPI_CPOL_Low,SPI_CPHA_1Edge,SPI_FirstBit_MSB,8);
	
	                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               
	/*IIC*/ 
	IIC_Init();
	
	/*USART1*/
	usart1_init();
	usart2_init();
	usart3_init();
	
//	/*ultrasound*/
//	H_GPIO_Init(B,9,GPIO_Speed_50MHz,GPIO_Mode_Out_PP);	//Trig
//	H_GPIO_Init(B,8,GPIO_Speed_50MHz,GPIO_Mode_IPD);	 //Echo
//	ULTRA_TRIG = 0;
//	H_EXTI_Init(B,8, EXTI_Trigger_Rising_Falling);		
//	H_EXTI_IT_ENABLE(9_5,0,1);
	
	
	/*TIM1 500Hz*/
	H_TIM18_ENRCC(1);
	H_TIMEBASE_Init(1,72,2000);			//500Hz
	H_TIMEBASE18_IT_UPDATE(1,1,2);
	H_TIMEBASE_START(1);
}



void TIM1_UP_IRQHandler(void)
{
	if (H_TIMEBASE_ISUPDATE_F(1))
	{
		scheduler_timer_event();
		H_TIMEBASE_CLEARUPDATE_F(1);
	}
}



//ultra
//void EXTI9_5_IRQHandler(void)
//{
//	static uint32_t tStart;
//	if(H_EXTI_ISPEND_F(8))
//	{
//		if(ULTRA_ECHO == 1)		//上升沿
//		{
//			tStart = micros();
//		}
//		else
//		{
//		//	ultrasound_read( micros() - tStart);
//		}
//		H_EXTI_CLEARPEND_F(8);
//	}
//	
//}

