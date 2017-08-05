#ifndef _WOS_H
#define _WOS_H
#include "sys.h"


/////////////////////////////////////////////////////
//注释此宏关闭时间记录,
////////////////////////////////////////////////////
//#define WOS_USE_TIMERECORD	


typedef void(*pOSFunc_t)(void);
#define WOS_THREAD_MAX 5			//不要设置超过127

int8_t  wos_add(pOSFunc_t pfunc, uint16_t frequency);
void wos_remove(uint8_t id);
void wos_run(void);
void wos_update(void);
void wos_init(pOSFunc_t mainfun);
	
#ifdef WOS_USE_TIMERECORD
uint32_t  wos_period(uint8_t  id);		//获取函数调用周期
uint32_t wos_time(uint8_t id);			//获取函数执行时间
#endif


#endif
