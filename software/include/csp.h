#ifndef _CSP_H
#define _CSP_H
#include "sys.h"




#define CSP_NOP					0							//空命令

//1-49 控制命令
#define CSP_ARM_IT					5					//上锁
#define CSP_DISARM_IT				6					//解锁
#define CSP_LAND_DOWN 			7					//自动下降
#define CSP_HOLD_ALT 				8					//定高
#define CSP_STOP_HOLD_ALT 	9					//取消定高
#define CSP_HEAD_FREE 			10				//无头模式
#define CSP_STOP_HEAD_FREE	11				//取消无头模式
#define CSP_POS_HOLD 				12				//悬停
#define CSP_STOP_POS_HOLD 	13				//取消悬停

#define CSP_ACC_CALI				14				//加速度校准
#define CSP_GYRO_CALI				15				//加速度校准
#define CSP_SENSOR_CALI			16				//加速度校准

#define CSP_CHANNEL_VALUE		17				//发送通道数据



//50-69 请求数据命令
#define CSP_ASK_STATE				50				//请求飞控状态



//70-99 设置命令
#define CSP_SET_ROLLERR 		70				//设置ROLL偏移
#define CSP_SET_PITCHERR	 	71				//设置PITCH偏移

#define CSP_PID1						72
#define CSP_PID2						73
#define CSP_PID3						74
#define CSP_PID4						75
#define CSP_PID5						76
#define CSP_PID6						77
#define CSP_PID7						78
#define CSP_PID8						79


//100-119 IAP(程序更新)命令
#define CSP_RES_SUCC		100
#define CSP_RES_FAIL		101
#define CSP_CODESIZE		102
#define CSP_CODEPACK		103
#define CSP_LOADFINISH	104
#define CSP_PACKCOUNT 	105
#define CSP_STARTIAP 		106
#define CSP_CHECK				108


//120-199,其他命令命令

#define CSP_ACCEL_DATA 120
#define CSP_CCD_DATA	 121
#define CSP_EAGLE_DATA 122
#define CSP_EAGLE_ROUTE 123





void csp_uploadUav(void);
void csp_uploadPC(void);

void csp_setName(u8 name);
void csp_put8(u8 dat);
void csp_put16(u16 dat);
void csp_put32(u32 dat);
void csp_putFloat(float dat);
void csp_putBuf(u8 *buf,u8 len);
void csp_clearBuffer(void);
void csp_recvByte(u8 dat);
void csp_sendCCD(uint8_t start,uint8_t mid,uint8_t end);
#endif

