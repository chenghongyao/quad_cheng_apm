#include "ano.h"
#include "sys.h"

#include "stdarg.h"
#include "stdlib.h"



//调用前先强制转换格式!!!
#define ANO_SetName(_name)	do{ano_txbuffer[2] = (_name);ano_txindex = 4;}while(0)
#define ANO_Put8(_dat)		do{ano_txbuffer[ano_txindex++] = (_dat);}while(0)
#define ANO_Put16(_dat)		do{	ano_txbuffer[ano_txindex++] = BYTE1((_dat));	ano_txbuffer[ano_txindex++] = BYTE0((_dat));}while(0)
#define ANO_Put32(_dat)		do{	ano_txbuffer[ano_txindex++] = BYTE3((_dat));	\
								ano_txbuffer[ano_txindex++] = BYTE2((_dat));	\
								ano_txbuffer[ano_txindex++] = BYTE1((_dat));	\
								ano_txbuffer[ano_txindex++] = BYTE0((_dat));}while(0)

#if ANO_SENDTO_PC == 1
static uint8_t ano_txbuffer[50]={0xAA,0xAA};	//发送数据缓存
#else
static uint8_t ano_txbuffer[50] = { 0xAA, 0xAF };	//发送数据缓存
#endif
static uint8_t ano_rxbuffer[50] = { 0xAA, 0xAA };	//接收数据缓存
static uint8_t ano_txindex=4;


			

/////////////////////////////////////////////////////////////////////////////////////
//Send_Data函数是协议中所有发送数据功能使用到的发送函数
//移植时，用户应根据自身应用的情况，根据使用的通信方式，实现此函数
void ANO_DT_Send_Data(uint8_t *dataToSend , uint8_t length)
{
//	tcp_send(0,dataToSend,length);
	//myputbuf(ano_txbuffer,length);
	nrf24l01_sendPacket3(dataToSend,length);
		//nrf24l01_sendAckPacket(dataToSend,length);
}



//数据上传
void ANO_Upload(void)
{
	uint8_t i;
	uint8_t check=0;
	ano_txbuffer[3] = (ano_txindex - 4);
	for(i=0;i<ano_txindex;i++)
	{
		check += ano_txbuffer[i];
	}
	ano_txbuffer[ano_txindex++] = check;
	ANO_DT_Send_Data(ano_txbuffer, ano_txindex);
}



//发送姿态
void ANO_DT_Send_Status(float angle_rol, float angle_pit, float angle_yaw, int32_t alt, uint8_t fly_model, uint8_t armed)
{
	int16_t tempint16_t;	
	ANO_SetName(0x01);
	tempint16_t = angle_rol*100;
	ANO_Put16(tempint16_t);
	tempint16_t = angle_pit*100;
	ANO_Put16(tempint16_t);
	tempint16_t = angle_yaw*100;
	ANO_Put16(tempint16_t);
	
	alt *= 100;
	ANO_Put32(alt);
	ANO_Put8(fly_model);
	ANO_Put8(armed);
	ANO_Upload();
}

//传感器数据
void ANO_DT_Send_Sensor(int16_t a_x,int16_t a_y,int16_t a_z,int16_t g_x,int16_t g_y,int16_t g_z,int16_t m_x,int16_t m_y,int16_t m_z)
{
	ANO_SetName(0x02);
	ANO_Put16(a_x);ANO_Put16(a_y);ANO_Put16(a_z);
	ANO_Put16(g_x);ANO_Put16(g_y);ANO_Put16(g_z);
	ANO_Put16(m_x);ANO_Put16(m_y);ANO_Put16(m_z);	
	
	ANO_Upload();
	
}

//遥控数据
void ANO_DT_Send_RCData(uint16_t thr,uint16_t yaw,uint16_t rol,uint16_t pit,uint16_t aux1,uint16_t aux2,uint16_t aux3,uint16_t aux4,uint16_t aux5,uint16_t aux6)
{
	ANO_SetName(0x03);
	ANO_Put16(thr);
	ANO_Put16(yaw);
	ANO_Put16(rol);
	ANO_Put16(pit);
	ANO_Put16(aux1);
	ANO_Put16(aux2);
	ANO_Put16(aux3);
	ANO_Put16(aux4);
	ANO_Put16(aux5);
	ANO_Put16(aux6);
	ANO_Upload();
}


//发送电压电流*100
void ANO_DT_Send_Power(uint16_t votage, uint16_t current)
{
	
	ANO_SetName(0x05);
	ANO_Put16(votage);
	ANO_Put16(current);
	ANO_Upload();
}



//电机动力:0-1000
void ANO_DT_Send_MotoPWM(uint16_t m_1,uint16_t m_2,uint16_t m_3,uint16_t m_4,uint16_t m_5,uint16_t m_6,uint16_t m_7,uint16_t m_8)
{
	ANO_SetName(0x06);
	ANO_Put16(m_1);
	ANO_Put16(m_2);
	ANO_Put16(m_3);
	ANO_Put16(m_4);
	ANO_Put16(m_5);
	ANO_Put16(m_6);
	ANO_Put16(m_7);
	ANO_Put16(m_8);
	ANO_Upload();
}


//发送pid,group为组编号,每3组PID为1组
void ANO_DT_Send_PID(uint8_t group,float p1_p,float p1_i,float p1_d,float p2_p,float p2_i,float p2_d,float p3_p,float p3_i,float p3_d)
{
	int16_t tempint16_t;	
	uint8_t t = 0x10 + group-1;
	ANO_SetName(t);
	
	tempint16_t = p1_p*100;ANO_Put16(tempint16_t);
	tempint16_t = p1_i*100;ANO_Put16(tempint16_t);
	tempint16_t = p1_d*100;ANO_Put16(tempint16_t);
	
	tempint16_t = p2_p*100;ANO_Put16(tempint16_t);
	tempint16_t = p2_i*100;ANO_Put16(tempint16_t);
	tempint16_t = p2_d*100;ANO_Put16(tempint16_t);
	
	tempint16_t = p3_p*100;ANO_Put16(tempint16_t);
	tempint16_t = p3_i*100;ANO_Put16(tempint16_t);
	tempint16_t = p3_d*100;ANO_Put16(tempint16_t);
	ANO_Upload();
}




//	
//	e-> uint8_t,s8
//	s-> uint16_t,int16_t
//	t-> uint32_t,int32_t
void ANO_SendCustomData(uint8_t name,const char *fmt,...)
{
	va_list ap;
	uint32_t temp;
	
	ANO_SetName(name);
	va_start(ap,fmt);
	for(;*fmt;fmt++)
	{
		temp = va_arg(ap,int);
		if(*fmt=='e')
		{
			ANO_Put8(temp);
		}
		else if(*fmt=='s')
		{
			ANO_Put16(temp);
		}
		else if(*fmt == 't')
		{
			ANO_Put32(temp);
		}
	}
	va_end(ap);
	ANO_Upload();
}






//extern void ANO_RecvData_CallBack(uint8_t cmd, uint8_t *buf, uint8_t len);
//void ANO_RecvByte(uint8_t dat)
//{
//	static uint8_t step = 0;
//	static uint8_t cmd;
//	static uint8_t len;
//	static uint8_t counter;
//	static uint8_t check;
//	switch(step)
//	{
//	case 0:
//		if (dat == 0xAA)
//		{
//			step++;
//			check = dat;
//		}
//		break;
//	case 1:
//		if (dat == 0xAF)
//		{
//			check  += dat;
//			step++;
//		}
//		else step = 0;
//		break;
//	case 2:
//		cmd = dat;
//		check += dat;
//		step++;
//		break;
//	case 3:
//		len = dat;
//		check += dat;
//		if (len == 0)step += 2;
//		else
//		{
//			counter = 0;
//			step++;
//		}
//		break;
//	case 4:
//		ano_rxbuffer[counter] = dat;
//		check += dat;
//		if (++counter==len)
//		{
//			step++;
//		}
//		break;
//	case 5:
//		if (check ==dat)
//		{
//			ANO_RecvData_CallBack(cmd,ano_rxbuffer,len);
//		}
//		step = 0;
//		break;
//	}
//}

//void ANO_RecvBytes(uint8_t *dat,uint16_t len)
//{
//	uint16_t  i;
//	for (i = 0; i < len ; i++)
//	{
//		ANO_RecvByte(*dat++);
//	}
//}

/////////////////////////////////////////////////////
//遥控用
////////////////////////////////////////////////////
void ANO_DT_Send_Control(int16_t THR,int16_t YAW,int16_t ROL,int16_t PIT,int16_t AUX1,int16_t AUX2,int16_t AUX3,int16_t AUX4)
{
	int16_t zero = 0;
	ANO_SetName(0x03);
	ANO_Put16(THR);
	ANO_Put16(YAW);
	ANO_Put16(ROL);
	ANO_Put16(PIT);
	ANO_Put16(AUX1);
	ANO_Put16(AUX2);
	ANO_Put16(AUX3);
	ANO_Put16(AUX4);
	ANO_Put16(zero);
	ANO_Put16(zero);
	ANO_Upload();
}

