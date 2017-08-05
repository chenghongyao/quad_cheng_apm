#include "hmc5883.h"
#include "hmc5883_dev.h"
#include "sys.h"



//底层驱动接口===============================================================
static void HMC5883_WriteByte(u8 reg_addr,u8 reg_dat)
{
	 IIC_DevWriteReg(HMC_IIC_ADDRESS,reg_addr, reg_dat);
}
static void HMC5883_ReadBuf(u8 reg_addr,u8* pBuf,u8 len)
{
	IIC_DevReadBuf(HMC_IIC_ADDRESS,reg_addr,len,pBuf);

}
//=================================================================




void hmc5883_init()
{
	//设置平均数和输出速率
	HMC5883_WriteByte(HMC_CONFIGA,HMC5883_AVRCOUNTS_8|HMC5883_SPEED_75|HMC5883_MS_NORMAL);
	HMC5883_WriteByte(HMC_CONFIGB, HMC5883_GN_1370);//设置量程 
	HMC5883_WriteByte(HMC_MODE, HMC5883_MR_ONE);//测量模式
}


//获取3个字节的ID: 'H','4','3'
void hmc5883_getID(uint8_t *id)
{
	HMC5883_ReadBuf(HMC_IDA,id,3);		
}
	



void hmc5883_getMagAdc(int16_t *mag)
{
	uint8_t magbuffer[6];
	HMC5883_ReadBuf(HMC_XMSB,magbuffer,6);		
	mag[0]=(int16_t)((magbuffer[0]<<8) | magbuffer[1]);
	mag[2]=(int16_t)((magbuffer[2]<<8) | magbuffer[3]);
	mag[1]=(int16_t)((magbuffer[4]<<8) | magbuffer[5]);
  HMC5883_WriteByte(HMC_MODE, HMC5883_MR_ONE);//测量模式
}

void hmc5883_getMag(float *mag)
{
	uint8_t magbuffer[6];
	HMC5883_ReadBuf(HMC_XMSB,magbuffer,6);		
	mag[0]= HMC5883_KMAG*(int16_t)((magbuffer[0]<<8) | magbuffer[1]);
	mag[2]= HMC5883_KMAG*(int16_t)((magbuffer[2]<<8) | magbuffer[3]);
	mag[1]= HMC5883_KMAG*(int16_t)((magbuffer[4]<<8) | magbuffer[5]);
	HMC5883_WriteByte(HMC_MODE, HMC5883_MR_ONE);//测量模式
}


