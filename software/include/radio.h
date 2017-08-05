#ifndef _RADIO_H_
#define _RADIO_H_
#include "sys.h"

void init_rc_in(void);
void init_rc_out(void);
void read_radio(void);
uint8_t rcin_rew_input(void);
void rcin_read(uint16_t *periods,uint8_t len);
#endif

