#ifndef _ALT_H_
#define _ALT_H_
#include "sys.h"

#include "filter.h"









typedef struct
{
	float height;		//高度,cm

	float height_old;
	float height_delta;
	uint8_t mesure_status;
}height_sensor_t;


//高度数据
typedef struct
{
	float height_raw;				//高度,mm
	float height_lpf;		//滤波后高度
	float speed;
	float acc;	
	float height_old;
	float speed_old;
}height_data_t;




//距离单位mm
typedef struct
{
	float est_acc_old;
	_filter_1_st fusion_acceleration;
	_filter_1_st fusion_speed_m;
	_filter_1_st fusion_speed_me;	//ÈÚºÏËÙ¶ÈºÍÔ­Ê¼ËÙ¶È²îÖµ
	_filter_1_st fusion_displacement;
	
	float acc;
	float speed;
	float displacement;
}height_fusion_t;




typedef struct
{
	float thr_take_off;		//起飞油门
	float speed_exp;
	
	float m_acc;		//加速度计测量加速度
	float m_speed;	//融合速度,加速度计积分,超声波融合,气压计融合,最后超声波与气压计融合
	float m_height;	//气压计测量高度
	float fusion_acc;	//气压计融合加速度
	float fusion_speed;//融合(测量??)速度
	float fusion_height;//气压计融合高度
	float fusion_baro_speed;//融合(测量??)速度
	
	
}alt_t;


extern height_sensor_t sensor_ultra;			//超声波高度数据
extern height_sensor_t sensor_baro;			//超声波高度数据

extern height_data_t height_sanor;
extern height_data_t height_baro;
extern alt_t mAlt;
float alt_ctrl(float dt,float thr,uint8_t ready);
void alt_init(void);
float alt_update(float dt);

#endif
