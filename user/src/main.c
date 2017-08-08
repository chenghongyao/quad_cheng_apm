#include "sys.h"

#include "sbus.h"


extern void setup(void);
extern void loop(void);
//============================
int main(void)
{	
	board_setup();
	delay_ms(400);			//磁力计上电后需要稳定时间
	motor_init();				
	nrf24l01_init();
	nrf24l01_setTxRxMode(NRF_TXEN);
	
	//ms5611_init();
	sbus_init();
	setup();
	while(1)
	{
		loop();
	}
}



