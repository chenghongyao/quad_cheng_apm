#ifndef _AP_STATE_H_
#define	_AP_STATE_H_

#include "sys.h"

void set_auto_armed(uint8_t b);
void set_pre_arm_check(uint8_t b);
void set_pre_arm_rc_check(uint8_t b);

void set_land_complete(uint8_t b);
void set_land_complete_maybe(uint8_t b);
#endif

