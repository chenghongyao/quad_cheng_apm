#include "sbus.h"
#include "sys.h"
#include "cmath.h"



sbus_t sbus;






void sbus_init(void)
{
	sbus.last_update = 0;
	sbus.rx_cnt = 0;
}



