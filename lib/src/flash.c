#include "flash.h"
#include "sys.h"



/////////////////////////////////////////////////////
//写之前将擦除改扇区
//len不能超过该扇区 地址为0x400或0x800的整数倍
////////////////////////////////////////////////////
void flash_write(uint32_t address, void *buf,uint16_t len)
{
	__IO uint16_t *pbuf = (uint16_t *)buf;
	uint16_t i;

	FLASH_Unlock();
	FLASH_ErasePage(address);
	for (i = 0; i < len/2 ; i++)
	{
		FLASH_ProgramHalfWord(address,*pbuf++);
		address+=2;
	}
	FLASH_Lock();
}

void flash_read(uint32_t address, void *buf,uint16_t len)
{
	__IO uint16_t *pbuf = (uint16_t *)buf;
	uint16_t i;

	for (i = 0; i < len/2 ; i++)
	{
		*pbuf++ = *(vu16*)address;
		address += 2;
	}
}




