#pragma once
#include "sys.h"



int16_t get_pilot_desired_throttle(int16_t throttle_control);
void get_pilot_desired_lean_angles(int16_t roll_in, int16_t pitch_in, int16_t *roll_out, int16_t *pitch_out);
float get_pilot_desired_yaw_rate(int16_t stick_angle);
float get_smoothing_gain(void);//2-12
void update_simple_mode(void);
void set_throttle_takeoff(void);
int16_t get_throttle_pre_takeoff(int16_t throttle_control);
int16_t get_pilot_desired_climb_rate(int16_t throttle_control);

