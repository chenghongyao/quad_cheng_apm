#ifndef NRF__H
#define NRF__H

#include "sys.h"


#define NRF_ACKPAY_ENABLE 

#ifdef NRF_ACKPAY_ENABLE
extern u8 nrf_ackbuf[32];
extern u8 nrf_acklen;			//当acklen=0时没有收到数据,读取完数据必须将其设置为0
#endif





#define MAX_TX 	0x10//重发次数达到上限
#define TX_OK	  0x20//发送完成
#define RX_OK		0x40//接收完成


enum{	NRF_TXEN=0,	NRF_RXEN=1,TXRX_OFF=2};
enum{	NRF_BR_1M=0,NRF_BR_2M=1,NRF_BR_250K=2};
enum{	NRF_POWER_M18dB=0,NRF_POWER_M12dB=1,NRF_POWER_M6dB=2,NRF_POWER_0dB=3};

//掩码
//6个通道
#define NRF_P0	0x01
#define NRF_P1	0x02
#define NRF_P2 	0x04
#define NRF_P3 	0x08
#define NRF_P4	0x10
#define NRF_P5 	0x20

//CONFIG
#define NRF_CONFIG_RXDR		0x40
#define NRF_CONFIG_TXDS		0x20
#define NRF_CONFIG_MAXRT	0x10
#define NRF_CONFIG_ENRCC	0x08
#define NRF_CONFIG_CRCO		0x04
#define NRF_CONFIG_PWRUP	0x02
#define NRF_CONFIG_PRIMRX	0x01
//SETUP_AW
#define NRF_AW_3BYTE 0x01
#define NRF_AW_4BYTE 0x02
#define NRF_AW_5BYTE 0x03

//FEATURE
#define NRF_EN_DPL			0x04			//使能自动数据宽度
#define NRF_EN_ACK_PAY	0x02			//使能应答包
#define NRF_EN_DYN_ACK 	0x01			//使能不应答









//============================================
uint8_t nrf24l01_FlushTx(void);
uint8_t nrf24l01_FlushRx(void);
uint8_t nrf24l01_getStatus(void);
void nrf24l01_clearStatus(void);
uint8_t nrf24l01_getDynamicPayloadSize(void);
uint8_t nrf24l01_setBitrate(uint8_t bitrate);
void nrf24l01_setPower(uint8_t pwr);
void nrf24l01_setTxAddr(uint8_t *addr,uint8_t len);
void nrf24l01_setRxAddr(uint8_t ch,uint8_t *addr,uint8_t len);	
void nrf24l01_setRxPW(uint8_t ch,uint8_t width);
void nrf24l01_setAddressWidth(uint8_t width);
void nrf_setRFCH(uint8_t ch);
void nrf24l01_setRetry(uint8_t count,uint8_t delay);
void nrf24l01_enableAck(uint8_t ch,uint8_t en);
void nrf24l01_enaleRxAddr(uint8_t ch);
void nrf24l01_enableDyn(uint8_t ch);
void nrf24l01_setFeature(uint8_t fea);
uint8_t nrf24l01_writePayload(uint8_t *dat,uint8_t length);
uint8_t nrf24l01_readPayload(uint8_t *dat,uint8_t length);




void nrf24l01_init(void);
uint8_t nrf24l01_check(void);
void nrf24l01_setTxRxMode(uint8_t mode);
uint8_t nrf24l01_sendPacket(void *txbuf,uint8_t len);
uint8_t nrf24l01_sendPacket2(void *txbuf,uint8_t len);
u8 nrf24l01_sendPacket3(void *txbuf,u8 len);
uint8_t nrf24l01_recvPacket(void *rxbuf);
void nrf24l01_sendAckPacket(uint8_t *buf,uint8_t len);
#endif

