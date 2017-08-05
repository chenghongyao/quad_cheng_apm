#ifndef _FLASH_H
#define _FLASH_H
#include "sys.h"

#define STM32_FLASH_SIZE 64						//64KFLASH
//扇区大小
#if STM32_FLASH_SIZE<256
#define STM_SECTOR_SIZE 1024
#else
#define STM_SECTOR_SIZE 2048
#endif


#define STM32_FLASH_BASE 0x08000000
#define STM32_FLASH_LASTPAGE	(STM32_FLASH_BASE+STM32_FLASH_SIZE*1024 - STM_SECTOR_SIZE)		//最后一页地址
#define STM32_EEPROM_ADDRESS (STM32_FLASH_BASE + STM_SECTOR_SIZE*20)

void flash_write(uint32_t address, void *buf, uint16_t len);
void flash_read(uint32_t address, void *buf, uint16_t len);

#endif
