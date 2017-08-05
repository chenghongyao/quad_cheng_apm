#ifndef _POS_H_
#define _POS_H_
#include "filter.h"

typedef struct
{
	float pos_raw;				
	float pos_lpf;	
	float speed;
	float acc;	
	float pos_old;
	float speed_old;
}pos_data_t;




//????mm
typedef struct
{
	float est_acc_old;

	_filter_1_st fusion_acceleration;
	_filter_1_st fusion_speed_m;
	_filter_1_st fusion_speed_me;	
	_filter_1_st fusion_displacement;
}pos_fusion_t;



typedef struct
{
	float x;
	float y;
	
	float vx;
	float vy;
	
	float vx_exp;
	float vy_exp;
	
	
}pos_t;



extern pos_t mPos;
extern pos_data_t posy;
extern pos_fusion_t fusion_posx;
extern pos_fusion_t fusion_posy;

void pos_prepare(float dt,float fc,float deadzone,float xn,pos_data_t *pos);
void pos_filter(float dt,_f_set_st *set,float est_acc,pos_data_t *pre_pos,pos_fusion_t *fusion_data);
void pos_update(float dt);
void pos_init(void);
void pos_ctrl(float dt);
#endif

