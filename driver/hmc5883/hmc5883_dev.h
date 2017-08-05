#ifndef _HMC5883_DEV_H
#define _HMC5883_DEV_H
#include "sys.h"

//寄存器地址
#define HMC_CONFIGA		0x00	//设置数据输出速率和测量配置MA输出平均数,DO输出速率,MS测量流程
#define HMC_CONFIGB		0x01	//设置增益(量程)
#define HMC_MODE		0x02
#define HMC_XMSB		0x03
#define HMC_XLSB		0x04
#define HMC_ZMSB		0x05
#define HMC_ZLSB		0x06
#define HMC_YMSB		0x07
#define HMC_YLSB		0x08
#define HMC_STATUS		0x09
#define HMC_IDA			0x0A
#define HMC_IDB			0x0B
#define HMC_IDC			0x0C

//=============================================



/*HMC_CONFIGA*/
//每次测量输出中选择的平均数
#define HMC5883_AVRCOUNTS_1	(0<<5)
#define HMC5883_AVRCOUNTS_2	(1<<5)
#define HMC5883_AVRCOUNTS_4	(2<<5)
#define HMC5883_AVRCOUNTS_8 (3<<5)
//输出速率
#define HMC5883_SPEED_0_75 	(0<<2)
#define HMC5883_SPEED_1_5 	(1<<2)
#define HMC5883_SPEED_3 		(2<<2)
#define HMC5883_SPEED_7_5 	(3<<2)
#define HMC5883_SPEED_15 		(4<<2)
#define HMC5883_SPEED_30 		(5<<2)
#define HMC5883_SPEED_75 		(6<<2)
//测量配置
#define HMC5883_MS_NORMAL 		0
#define HMC5883_MS_POSTIVE 		1
#define HMC5883_MS_NEGATIVE		2


/*HMC_CONFIGB*/
#define HMC5883_GN_1370	0x00	//0.88Ga -推荐最大测量范围
#define HMC5883_GN_1090	0x01	//1.3Ga
#define HMC5883_GN_820	0x02	//1.9Ga
#define HMC5883_GN_660	0x03	//2.5Ga
#define HMC5883_GN_440	0x04	//4.0Ga
#define HMC5883_GN_390	0x05	//4.7Ga
#define HMC5883_GN_330	0x06	//5.6Ga
#define HMC5883_GN_230	0x07	//8.1Ga

/*HMC_MODE*/
#define HMC5883_MR_COUNTINUS	0x00		//连续测量
#define HMC5883_MR_ONE				0x01		//单次测量
#define HMC5883_MR_IDLE				0x00		//闲置模式

/*HMC STATUS*/
#define HMC5883_SR_LOCK				0x02		//
#define HMC5883_SR_RDY				0x01		//



#define HMC5883_MAGADC_MAX	2047			//-2048-2047,超出该值时,输出为4096	


#define  HMC_IIC_ADDRESS 0x3C



#endif
