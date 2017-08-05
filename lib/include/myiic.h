#ifndef MYIIC_H
#define MYIIC_H

#include "sys.h"




//===========================

void IIC_Init(void);
void IIC_Start(void);
void IIC_Stop(void);
void IIC_WriteByte(uint8_t byte);
uint8_t IIC_ReadByte(uint8_t ack);
uint8_t IIC_WaitAck(void);
void IIC_Ack(void);
void IIC_NAck(void);



uint8_t IIC_DevWriteByte(uint8_t dev_addr,uint8_t val);
uint8_t IIC_DevWriteReg(uint8_t dev_addr,uint8_t reg_addr,uint8_t reg_val);
uint8_t IIC_DevReadReg(uint8_t dev_addr,uint8_t reg_addr);
uint8_t IIC_DevReadBuf(uint8_t dev_addr,uint8_t reg_addr,uint8_t len,uint8_t* pBuf);	//先读低字节

#endif

