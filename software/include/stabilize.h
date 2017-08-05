#ifndef _STABILIZE_H_
#define _STABILIZE_H_

#include "sys.h"





void update_simple_mode(void);
void get_pilot_desired_lean_angles(int16_t roll_in, int16_t pitch_in, int16_t *roll_out, int16_t *pitch_out);
float get_pilot_desired_yaw_rate(int16_t stick_angle);
int16_t get_pilot_desired_throttle(int16_t throttle_control);
float get_smoothing_gain(void);//2-12
void stabilize_run(void);

#endif

