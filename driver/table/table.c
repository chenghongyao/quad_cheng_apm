#include "table.h"
#include "sys.h"


#include "imu.h"
table_t mTab;


void table_load()
{
	flash_read(STM32_FLASH_LASTPAGE,(u16*)&mTab,(sizeof(mTab)+1)/2);

		{
		
		mTab.TableInited = 'Y';
		//写入flash
		flash_write(STM32_FLASH_LASTPAGE,(u16*)&mTab,(sizeof(mTab)+1)/2);
	}
}



void table_save()
{
	
	mTab.acc_offset[0] = mImu.accel_offset[0];
	mTab.acc_offset[1] = mImu.accel_offset[1];
	mTab.acc_offset[2] = mImu.accel_offset[2];

	flash_write(STM32_FLASH_LASTPAGE,(u16*)&mTab,sizeof(mTab)/2);
}
