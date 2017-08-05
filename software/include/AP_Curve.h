#ifndef AP_CURVE_H_
#define  AP_CURVE_H_

#include "sys.h"


typedef struct
{
	int16_t x[4];
	int16_t y[4];
	float slope[4];
	uint8_t num_points;
	
}AP_CurveInt16_Size4_T;

void AP_CurveInt16_Size4_Glear(AP_CurveInt16_Size4_T *curve);
uint8_t AP_CurveInt16_Size4_AddPoint(AP_CurveInt16_Size4_T *curve,int16_t x,int16_t y);
int16_t AP_CurveInt16_Size4_GetY(AP_CurveInt16_Size4_T *curve,int16_t x);


#endif

