#ifndef _LOWPASSFILTER_H_
#define _LOWPASSFILTER_H_
#include "sys.h"

typedef struct 
{
	float alpha;
	uint8_t base_value_set;
	float base_value;
}low_pass_fiter_t;


void low_pass_filter_set_fc(low_pass_fiter_t *lpf,float time_step, float cutoff_freq);
float low_pass_filter_apply(low_pass_fiter_t *lpf,float sample);
void low_pass_filter_reset(low_pass_fiter_t *lpf,float new_base_value);
#endif
