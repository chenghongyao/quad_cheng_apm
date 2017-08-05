#include "lib_helper.h"
#include "sys.h"



uint8_t SPI_RWByte(SPI_TypeDef *SPIx,uint8_t txc)
{
	while((SPIx->SR&SPI_SR_TXE)==0);
	SPIx->DR = txc;
	while((SPIx->SR&SPI_SR_RXNE)==0);
	return SPIx->DR;	
}

void H_SPI2_initPin()
{
	H_GPIO_Init(B, 13, GPIO_Speed_50MHz, GPIO_Mode_AF_PP);//SCK
	H_GPIO_Init(B, 15, GPIO_Speed_50MHz, GPIO_Mode_AF_PP);//MOSI
	H_GPIO_Init(B, 14, GPIO_Speed_50MHz, GPIO_Mode_IN_FLOATING);//MISO
}



void H_SPI3_initPin()
{
	JTAG_Set(SWD_ENABLE);			//禁用JTAG
	H_GPIO_Init(B,3,GPIO_Speed_50MHz,GPIO_Mode_AF_PP);
	H_GPIO_Init(B,4,GPIO_Speed_50MHz,GPIO_Mode_AF_PP);
	H_GPIO_Init(B,5,GPIO_Speed_50MHz,GPIO_Mode_AF_PP);
}


void capturepwm_init()
{
	TIM_ICInitTypeDef  TIM_ICInitStructure;
	
	H_GPIO_Init(B,6,GPIO_Speed_50MHz,GPIO_Mode_IN_FLOATING);
	
	H_TIMx_ENRCC(4);
	H_TIMEBASE_Init(4,72,40000);
	
	
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
	TIM_ICInitStructure.TIM_ICFilter = 0x0;       /*滤波 0x00~0x0F*/
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;			//上升沿有效,测量周期,复位计数器
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;   
	TIM_PWMIConfig(TIM4, &TIM_ICInitStructure); 

	//从模式控制器配置
	TIM_SelectInputTrigger(TIM4, TIM_TS_TI1FP1);     //选择有效输入端 	
	TIM_SelectSlaveMode(TIM4, TIM_SlaveMode_Reset);  //从模式控制器设置为复位模式
  TIM_SelectMasterSlaveMode(TIM4, TIM_MasterSlaveMode_Enable);       	  
	H_TIMEBASE_START(4);
	//==========================
	H_GPIO_Init(A,3,GPIO_Speed_50MHz,GPIO_Mode_IN_FLOATING);
	
	H_TIMx_ENRCC(2);
	H_TIMEBASE_Init(2,72,40000);
	
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_4;
	TIM_ICInitStructure.TIM_ICFilter = 0x0;       /*滤波 0x00~0x0F*/
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;			//上升沿有效,测量周期,复位计数器
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;   
	TIM_PWMIConfig(TIM2, &TIM_ICInitStructure); 

	//从模式控制器配置
	TIM_SelectInputTrigger(TIM2, TIM_TS_TI2FP2);     //选择有效输入端 	
	TIM_SelectSlaveMode(TIM2, TIM_SlaveMode_Reset);  //从模式控制器设置为复位模式
  TIM_SelectMasterSlaveMode(TIM2, TIM_MasterSlaveMode_Enable);       	  
	H_TIMEBASE_START(2);
	
}


/*
#define TM2_CH1 TIM2->CCR1
#define TM2_CH2 TIM2->CCR2
#define TM2_CH3 TIM2->CCR3
#define TM2_CH4 TIM2->CCR4

#define PIN_AILE	PAin(3)


void capture_init()
{
	TIM_ICInitTypeDef  TIM_ICInitStructure;
	
	H_GPIO_Init(A,3,GPIO_Speed_50MHz,GPIO_Mode_IPD);
	H_GPIO_Init(A,2,GPIO_Speed_50MHz,GPIO_Mode_IPD);
	H_GPIO_Init(A,1,GPIO_Speed_50MHz,GPIO_Mode_IPD);
	H_GPIO_Init(A,0,GPIO_Speed_50MHz,GPIO_Mode_IPD);
	
	H_TIMx_ENRCC(2);
	H_TIMEBASE_Init(2,72,0xFFFF);
	
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_4;
	TIM_ICInitStructure.TIM_ICFilter = 0x0;      
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;			//上升沿有效,测量周期,复位计数器
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1; 
	TIM_ICInit(TIM2,&TIM_ICInitStructure);
	
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_3;
	TIM_ICInit(TIM2,&TIM_ICInitStructure);
		TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
	TIM_ICInit(TIM2,&TIM_ICInitStructure);
		TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
	TIM_ICInit(TIM2,&TIM_ICInitStructure);
	
	
	TIM_ITConfig(TIM2, TIM_IT_CC1, ENABLE);
	TIM_ITConfig(TIM2, TIM_IT_CC2, ENABLE);
	TIM_ITConfig(TIM2, TIM_IT_CC3, ENABLE);
	TIM_ITConfig(TIM2, TIM_IT_CC4, ENABLE);
	
	H_NVIC_INIT(TIM2,0,3);
	H_TIMEBASE_START(2);
}

void TIM2_IRQHandler(void)
{
	static uint8_t  capturelaststa = 0;
	static uint16_t capturenum1[4];
	uint16_t capturenum2[4];
	uint16_t capturenum[4];

	


	if(TIM_GetITStatus(TIM2,TIM_IT_CC1) == SET) 
	{
		TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);	
		
		if((capturelaststa&0x01) == 0)//上升沿
		{
			capturenum1[0] = TM2_CH1;
			TIM_OC1PolarityConfig(TIM2, TIM_ICPolarity_Falling); //设置为下降沿捕获		  
			capturelaststa |= 0x01;
		}
		else	//下降沿
		{
			capturenum2[0] = TM2_CH1;
			TIM_OC1PolarityConfig(TIM2, TIM_ICPolarity_Rising); //设置为上升沿捕获		
			capturenum[0] = (capturenum2[0]>capturenum1[0])?(capturenum2[0]-capturenum1[0]):(0xFFFF-capturenum1[0]+capturenum2[0]);
			
//		debug("ch1=%d\n",capturenum[0]);
			capturelaststa &= ~0x01;
		}
	}
	
	if(TIM_GetITStatus(TIM2,TIM_IT_CC2) == SET) 
	{
		TIM_ClearITPendingBit(TIM2, TIM_IT_CC2);	
		
		if((capturelaststa&0x02) == 0)//上升沿
		{
			capturenum1[1] = TM2_CH2;
			TIM_OC2PolarityConfig(TIM2, TIM_ICPolarity_Falling); //设置为下降沿捕获		  
			capturelaststa |= 0x02;
		}
		else	//下降沿
		{
			capturenum2[1] = TM2_CH2;
			TIM_OC2PolarityConfig(TIM2, TIM_ICPolarity_Rising); //设置为上升沿捕获		
			capturenum[1] = (capturenum2[1]>capturenum1[1])?(capturenum2[1]-capturenum1[1]):(0xFFFF-capturenum1[1]+capturenum2[1]);
	//		debug("ch2=%d\n",capturenum[1]);
			capturelaststa &= ~0x02;
		}
	}

	
	if(TIM_GetITStatus(TIM2,TIM_IT_CC3) == SET) 
	{
		TIM_ClearITPendingBit(TIM2, TIM_IT_CC3);	
		
		if((capturelaststa&0x04) == 0)//上升沿
		{
			capturenum1[2] = TM2_CH3;
			TIM_OC3PolarityConfig(TIM2, TIM_ICPolarity_Falling); //设置为下降沿捕获		  
			capturelaststa |= 0x04;
		}
		else	//下降沿
		{
			capturenum2[2] = TM2_CH3;
			TIM_OC3PolarityConfig(TIM2, TIM_ICPolarity_Rising); //设置为上升沿捕获		
			capturenum[2] = (capturenum2[2]>capturenum1[2])?(capturenum2[2]-capturenum1[2]):(0xFFFF-capturenum1[2]+capturenum2[2]);
			debug("ch3=%d\n",capturenum[2]);
			capturelaststa &= ~0x04;
		}
	}


	if(TIM_GetITStatus(TIM2,TIM_IT_CC4) == SET) 
	{
		TIM_ClearITPendingBit(TIM2, TIM_IT_CC4);	
		
		if((capturelaststa&0x08) == 0)//上升沿
		{
			capturenum1[3] = TM2_CH4;
			TIM_OC4PolarityConfig(TIM2, TIM_ICPolarity_Falling); //设置为下降沿捕获		  
			capturelaststa |= 0x08;
		}
		else	//下降沿
		{
			capturenum2[3] = TM2_CH4;
			TIM_OC4PolarityConfig(TIM2, TIM_ICPolarity_Rising); //设置为上升沿捕获		
			capturenum[3] = (capturenum2[3]>capturenum1[3])?(capturenum2[3]-capturenum1[3]):(0xFFFF-capturenum1[3]+capturenum2[3]);
			debug("ch4=%d\n",capturenum[3]);
			capturelaststa &= ~0x08;
		}
	}
	
//	debug("ch1=%d,ch2=%d,ch3=%d,ch4=%d\n",capturenum[0],capturenum[1],capturenum[2],capturenum[3]);
}
*/
