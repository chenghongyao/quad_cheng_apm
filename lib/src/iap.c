#include "iap.h"
#include "sys.h"

typedef void(*iapfun)(void);
iapfun jumpapp2;

void iap_run(uint32_t appaddr)
{
	if (((*(vu32*)appaddr) & 0x2FFE0000) == 0x20000000) //妫€鏌ユ爤椤舵槸鍚﹀悎娉?
	{
		jumpapp2 = (iapfun)*(vu32*)(appaddr + 4);		//鑾峰彇澶嶄綅鍚戦噺		
		//MSR_MSP(*(vu32*)appaddr);									//鍒濆鍖朅PP鍫嗘爤鎸囬拡		
		__set_MSP(*(vu32*)appaddr);
		jumpapp2(); 
	}
}
