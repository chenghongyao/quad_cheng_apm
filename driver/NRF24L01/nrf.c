#include "nrf_dev.h"
#include "nrf.h"



//#include "delay.h"
//#include "spi.h"
//#include "led.h"
//#include "usart.h"





static u8 f_tx_wait = 0;
static u8 rf_setup;
static u8 packet_size=32;
static u8 rfch = 48;
static u8 bitrate = NRF_BR_2M;
static u8 rfpower = NRF_POWER_0dB;
static u8 txrx_address[5] = { 'w', 'e', 'i', 'c', 'h' };  //发送地址


#ifdef NRF_ACKPAY_ENABLE
u8 nrf_ackbuf[32];
u8 nrf_acklen=0;
#endif






void nrf24l01_init()
{
	nrf24l01_setTxRxMode(TXRX_OFF);
	nrf24l01_setTxAddr(txrx_address,5);							//发送地址	
	nrf24l01_setRxAddr(RX_ADDR_P0,txrx_address,5);	//通道0接收地址
	nrf24l01_enaleRxAddr(NRF_P0);									//使能通道0	
#ifdef NRF_ACKPAY_ENABLE
	nrf24l01_enableAck(NRF_P0,1);								
	nrf24l01_setRetry(10,1);
	//使能动态数据宽度,应答包
	nrf24l01_setFeature(NRF_EN_DPL|NRF_EN_ACK_PAY);
	nrf24l01_enableDyn(NRF_P0);	
#else
	nrf24l01_enableAck(NRF_P0,0);
	nrf24l01_setRxPW(RX_PW_P0,packet_size);			//数据宽度,必须与发送一致
#endif
	
	nrf24l01_setBitrate(bitrate);									
	nrf24l01_setPower(rfpower);									
	nrf_setRFCH(rfch);											
	nrf24l01_FlushTx();	nrf24l01_FlushRx();
	nrf24l01_clearStatus();
}



//========================================================
//以下程序无须改动
u8 nrf24l01_writeReg(u8 reg,u8 value)
{
	u8 status;
	NRF_CSN=0; //使能 SPI 传输
	status =NRF_ReadWriteByte(NRF_WRITE_REG|reg); //发送寄存器号
	NRF_ReadWriteByte(value); //写入寄存器的值
	NRF_CSN=1; //禁止 SPI 传输
	return(status); //返回状态值
}

u8 nrf24l01_writeRegMulti(u8 reg, u8 *pBuf, u8 len)
{
	u8 status,u8_ctr; 
	NRF_CSN = 0; //使能 SPI 传输
	status = NRF_ReadWriteByte(NRF_WRITE_REG|reg); //发送寄存器值(位置),并读取状态值
	for(u8_ctr=0; u8_ctr<len; u8_ctr++)
		NRF_ReadWriteByte(*pBuf++); //写入数据
	NRF_CSN = 1; //关闭 SPI 传输
	return status; //返回读到的状态值
} 


//读取 SPI 寄存器值
//reg:要读的寄存器
u8 nrf24l01_readReg(u8 reg)
{
	u8 reg_val; 
	NRF_CSN = 0; //使能 SPI 传输 
	NRF_ReadWriteByte(NRF_READ_REG|reg); //发送寄存器号
	reg_val=NRF_ReadWriteByte(0XFF); //读取寄存器内容
	NRF_CSN = 1; //禁止 SPI 传输 
	return(reg_val); //返回状态值
} 

u8 nrf24l01_readRegMulti(u8 reg, u8 *pBuf, u8 len)
{
	u8 status,u8_ctr; 
	NRF_CSN = 0; //使能 SPI 传输
	status = NRF_ReadWriteByte(NRF_READ_REG|reg); //发送寄存器值(位置),并读取状态值
	for(u8_ctr=0; u8_ctr<len; u8_ctr++)
		*pBuf++=NRF_ReadWriteByte(0xFF); //写入数据
	NRF_CSN = 1; //关闭 SPI 传输
	return status; //返回读到的状态值
} 





//用于测试总线是否可用,检测成功返回0
u8 nrf24l01_check(void)
{
	u8 buf[5]={0XA5,0XA5,0XA5,0XA5,0XA5};
	u8 i;
	nrf24l01_writeRegMulti(TX_ADDR,buf,5);//写入 5 个字节的地址.
	nrf24l01_readRegMulti(TX_ADDR,buf,5); //读出写入的地址 
	for(i=0;i<5;i++)
		if(buf[i]!=0XA5)return 1;		//进行对比
	return 0;
} 

static u8 Strobe(u8 state)		//写命令
{
	u8 res;
	NRF_CSN = 0; //使能 SPI 传输 
	res = NRF_ReadWriteByte(state);
	NRF_CSN = 1;	
	return res;
}
u8 nrf24l01_FlushTx()
{
	return Strobe(FLUSH_TX);
}
u8 nrf24l01_FlushRx()
{
	return Strobe(FLUSH_RX);
}
u8 nrf24l01_getStatus()
{
	return Strobe(NRF_NOP);
}
void nrf24l01_clearStatus()
{
	nrf24l01_writeReg(STATUS,0x70); //清除 TX_DS 或MAX_RT 中断标志

}
u8 nrf24l01_getDynamicPayloadSize()
{
	u8 res;
	NRF_CSN = 0; //使能 SPI 传输 
	NRF_ReadWriteByte(R_RX_PL_WID);
	res = NRF_ReadWriteByte(NRF_NOP);
	NRF_CSN = 1;
	return res;
}




//0 1Mbps
//1 2Mbps
//2 250kbps
u8 nrf24l01_setBitrate(u8 bitrate)
{
	rf_setup = (rf_setup & 0xD7) | ((bitrate & 0x02) << 5) | ((bitrate & 0x01) << 3);
	return nrf24l01_writeReg(RF_SETUP,rf_setup); //设置 TX 发射参数,0db增益,2Mbps,低噪声增益开启
}

//pwr: 0=-18dB  1=-12dB 2=-6dB 3=0dB
void nrf24l01_setPower(u8 pwr)
{
  rf_setup = (rf_setup & 0xF9) | ((pwr & 0x03) << 1);
	nrf24l01_writeReg(RF_SETUP,rf_setup); //设置 TX 发射参数,0db增益,2Mbps,低噪声增益开启
}


//设置发送地址
void nrf24l01_setTxAddr(u8 *addr,u8 len)
{	
	nrf24l01_setAddressWidth(len);
	nrf24l01_writeRegMulti(TX_ADDR,addr,len);	
}


//设置发送地址
void nrf24l01_setRxAddr(u8 ch,u8 *addr,u8 len)
{
	nrf24l01_setAddressWidth(len);
	nrf24l01_writeRegMulti(ch,addr,len);	
}


//设置接收有效数据宽度 0-32
//默认为0,通道未使用
//ch: RX_PW_P0-RX_PW_P5
void nrf24l01_setRxPW(u8 ch,u8 width)
{
	packet_size = width;
	nrf24l01_writeReg(ch,width&0x3f);	//选择接收通道0 的有效数据宽度
}

//3-5
void nrf24l01_setAddressWidth(u8 width)
{
	if (width > 5) width = 5;
  if (width < 3) width = 3;  
	nrf24l01_writeReg(SETUP_AW,width-2);		//设置地址宽度
}


//设置频道 0-127
void nrf_setRFCH(u8 ch)
{
	nrf24l01_writeReg(RF_CH,ch); //设置 RF 通道
}

//设置重发
//count:重发次数0-15,为0时禁止重发
//delay:重发时间0-15, 0为250us,每位加250us
void nrf24l01_setRetry(u8 count,u8 delay)
{
	nrf24l01_writeReg(SETUP_RETR,((delay<<4)&0xF0) | (count&0x0F));
}


//使能应答
//ch：NRF_P0-NRF_P5 ，可用或运算
void nrf24l01_enableAck(u8 ch,u8 en)
{
	if(en)
	{
		nrf24l01_writeReg(EN_AA,ch); //使能通道 0 的自动应答
	}
	else
	{
		nrf24l01_writeReg(EN_AA,ch^ch); //使能通道 0 的自动应答
	}
}
//NRF_P0-NRF_P5 ,可用或运算
void nrf24l01_enaleRxAddr(u8 ch)
{
	nrf24l01_writeReg(EN_RXADDR,ch); //使能通道0的接收地址
}



//NRF_P0-NRF_P5 ,可用或运算
void nrf24l01_enableDyn(u8 ch)
{
	nrf24l01_writeReg(DYNPD,ch);		//使能通道0的动态数据宽度(带数据包应答的功能所必须)
}

//使能动态数据宽度,应答报..
void nrf24l01_setFeature(u8 fea)
{
	nrf24l01_writeReg(FEATURE,fea);	
}


u8 nrf24l01_writePayload(u8 *dat,u8 length)
{
	u8 res,i;
	NRF_CSN = 0; //使能 SPI 传输 
	
	res	= NRF_ReadWriteByte(WR_TX_PLOAD); //发送寄存器值(位置),并读取状态值
	for(i=0;i<length;i++)
	{
		NRF_ReadWriteByte(*dat++);
	}
	
	NRF_CSN = 1;
	return res;
}

u8 nrf24l01_readPayload(u8 *dat,u8 length)
{
	u8 res,i;
	NRF_CSN = 0; //使能 SPI 传输 
	res	= NRF_ReadWriteByte(RD_RX_PLOAD); //发送寄存器值(位置),并读取状态值
	for(i=0;i<length;i++)
	{
		*dat++=NRF_ReadWriteByte(0xFF);
	}
	NRF_CSN = 1;	
	return res;
}

//============================================================




void nrf24l01_sendAckPacket(u8 *buf,u8 len)
{
	u8 i;	
	NRF_CSN = 0; //使能 SPI 传输 
	NRF_ReadWriteByte(W_ACK_PAYLOAD); //发送寄存器值(位置),并读取状态值
	for(i=0;i<len;i++)
	{
		NRF_ReadWriteByte(*buf++);
	}
	NRF_CSN = 1;	
}

//32字节
u8 nrf24l01_sendPacket(void *txbuf,u8 len)
{
	u8 sta;
	nrf24l01_writePayload(txbuf,len);//写数据到TX BUF 32 个字节
	while(NRF_IRQ == 1); 				//等待发送完成
	sta=nrf24l01_readReg(STATUS); //读取状态寄存器的值 
	nrf24l01_writeReg(STATUS,sta); //清除 TX_DS 或MAX_RT 中断标志
	
#ifdef NRF_ACKPAY_ENABLE
	if(sta&RX_OK)//发送完成
	{
		nrf_acklen = nrf24l01_getDynamicPayloadSize();
		nrf24l01_readPayload(nrf_ackbuf,nrf_acklen);//读取数据	
		return RX_OK;
	}
	if(sta&MAX_TX) //达到最大重发次数
	{
		nrf24l01_FlushTx();
		return MAX_TX;
	}
#endif
	if(sta&TX_OK)//发送完成
	{
		return TX_OK;
	}
	return 0xff;//其他原因发送失败
}


//32字节,发送完成后立即返回
u8 nrf24l01_sendPacket2(void *txbuf,u8 len)
{
	u8 sta;
	if(f_tx_wait)
	{
		while(NRF_IRQ == 1); 				//等待发送完成
		sta=nrf24l01_readReg(STATUS); //读取状态寄存器的值 
		nrf24l01_writeReg(STATUS,sta); //清除 TX_DS 或MAX_RT 中断标志
#ifdef NRF_ACKPAY_ENABLE
		if(sta&RX_OK)//发送完成
		{
			nrf_acklen = nrf24l01_getDynamicPayloadSize();
			nrf24l01_readPayload(nrf_ackbuf,nrf_acklen);//读取数据	
			return RX_OK;
		}
		if(sta&MAX_TX) //达到最大重发次数
		{
			nrf24l01_FlushTx();
			return MAX_TX;
		}
#endif
	}
	
	nrf24l01_writePayload(txbuf,len);//写数据到TX BUF 32 个字节
	f_tx_wait = 1;
	return 0xff;//其他原因发送失败
}



//返回接收数据长度,错误返回0
u8 nrf24l01_recvPacket(void *rxbuf)
{
	if(!NRF_IRQ)
	{
#ifdef NRF_ACKPAY_ENABLE
		u8 rxlen;	
		rxlen = nrf24l01_getDynamicPayloadSize();
		if(rxlen>0)
		{
			nrf24l01_readPayload(rxbuf,rxlen);//读取数据	
			nrf24l01_clearStatus();		
			return rxlen;
		}
#else
		nrf24l01_readPayload(rxbuf,packet_size);//读取数据	
		nrf24l01_clearStatus();
		return packet_size;
#endif
		
	}
	return 0;//没收到任何数据	
}



void nrf24l01_setTxRxMode(u8 mode)
{	
	if(mode == NRF_RXEN)
	{
		NRF_CE = 0;
		//打开接收模式
		nrf24l01_writeReg(NRF_CONFIG,	NRF_CONFIG_ENRCC
																|NRF_CONFIG_CRCO
																|NRF_CONFIG_PWRUP
																|NRF_CONFIG_PRIMRX);
		
		nrf24l01_FlushTx();	nrf24l01_FlushRx();
		nrf24l01_clearStatus();
		NRF_CE = 1;
		delay_us(200);
	}
	else if(mode == NRF_TXEN)
	{
		NRF_CE = 0;
		nrf24l01_writeReg(NRF_CONFIG,NRF_CONFIG_ENRCC
																|NRF_CONFIG_CRCO
																|NRF_CONFIG_PWRUP);
		nrf24l01_FlushTx();	nrf24l01_FlushRx();
		nrf24l01_clearStatus();
		NRF_CE = 1;
		delay_us(200);
	}
	else
	{
		NRF_CE = 0;
		nrf24l01_writeReg(NRF_CONFIG,0);
		NRF_CE = 1;
		
	}
}







