#ifndef _FILTER_H
#define _FILTER_H
#include "sys.h"

#define FLITER_LPF_MAXORDER 10


//yn +=FILTER1ST_COFF(fc,dt)*[xn-y(n-1)]
#define FILTER1ST_COEF(fc,dt) (1.0f/(1.0f+1.0f/(6.28f*(dt)*(fc))))
#define FILTER1ST_COEF2(wc) (1.0f/(1.0f+1.0f/(3.1415f*(wc))))	//wc = 0~1,不能为0
#define FILTER_EASY_COEF(_fc,_dt) (2*3.14f*(_dt)*_fc)


//from ano
#define LPF_1_(hz,t,in,out) ((out) += ( 1 / ( 1 + 1 / ( (hz) *6.28f *(t) ) ) ) *( (in) - (out) ))

typedef struct
{
	float an[FLITER_LPF_MAXORDER];
	float bn[FLITER_LPF_MAXORDER];
	float wn[FLITER_LPF_MAXORDER];
	u8 order;
}filter_t;



typedef struct
{
	float w1;
	float w2;
	float a1;
	float a2;

	float b0;
	float b1;
	float b2;
}lpf2_t;


typedef struct
{
	//分别为加速度,速度,位置
	float b1;		//数据低通基础截止频率
	float b2;
	float b3;
	
	float g1;		//变化率低通频率
	float g2;
	float g3;

}_f_set_st;

typedef struct
{
	float a;			//变化量(低通滤波结果
	float b;			//变化量平方
	float e_nr;		//低通滤波有效率??作为基础截止频率的修正系数
	float out;		//滤波输出
} _filter_1_st;

void lpf_butter(lpf2_t *filter, float wc);
void lpf2nd_init(lpf2_t *filter, float b0, float b1, float b2, float a1, float a2);
float lpf2nd_next(lpf2_t *filter, float xn);
float lpf_next(filter_t *filter, float xn);

float filter_window(float xn, float *win, uint16_t *index, uint16_t size);
s32 Moving_Median(s32 moavarray[],u16 len ,u16 *fil_p,s32 in);
void anotc_filter_1(float base_hz,float gain_hz,float dT,float in,_filter_1_st *f1);
void Moving_Average(float moavarray[],u16 len ,u16 *fil_cnt,float in,float *out);
#endif
