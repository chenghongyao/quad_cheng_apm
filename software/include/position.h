#ifndef _POSITION_H_
#define _POSITION_H_




typedef struct
{
	float pos_raw;				
	float pos_lpf;	

	float speed;
	float acc;	
	float pos_old;
	float speed_old;
}pos_sensor_t;


extern pos_sensor_t pos_data_x;
void position_prepare(float dt,float fc,float deadzone,float xn,pos_sensor_t *pos);
void position_init(void);
#endif

