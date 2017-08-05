#ifndef _EAGLE_R_H_
#define _EAGLE_R_H_

#include "sys.h"



typedef enum
{	
	ROUTE_NONE = 0,		//没有特殊路线,直线??
	ROUTE_LEFT,				//左拐
	ROUTE_RIGHT,			//右拐
	ROUTE_CROSS,			//十字
	ROUTE_CIRCLE,			//圆形

}route_type_t;


typedef struct
{
	uint32_t dt;
	
	float distance;
	float turn_distance;
	
	float slope;
	uint8_t turn_type; 
	
	float yp;
	float xp;
		
	//摄像头参数
	float pitch;	//前瞻角,rad
	float f;		 //焦距，cm
	float dx;			//每个像素的长度
	float dy;
	
	float sin_pitch;
	float cos_pitch;
}eagler_t;




extern eagler_t mEagler;
void eagler_init(float theta_err);
void eagle_calixy(float xp,float yp,float *x,float *y,float h);
void eagle_calix(float xp,float *x,float h);
void eagle_caliy(float yp,float x,float *y,float h);


#endif

