#include "AP_Curve.h"
#include "sys.h"


void AP_CurveInt16_Size4_Glear(AP_CurveInt16_Size4_T *curve)
{
	uint16_t i;
	for(i=0;i<4;i++)
	{
		curve->x[i] = 0;
		curve->y[i] = 0;
		curve->slope[i] = 0;
	}
	curve->num_points = 0;
}

uint8_t AP_CurveInt16_Size4_AddPoint(AP_CurveInt16_Size4_T *curve,int16_t x,int16_t y)
{
	if(curve->num_points < 4 ) 
	{
		curve->x[curve->num_points] = x;
		curve->y[curve->num_points] = y;

		// increment the number of points
		// if we have at least two points calculate the slope
		if( curve->num_points > 0) 
		{
			//该点上条直线斜率
			curve->slope[curve->num_points-1] = (float)(curve->y[curve->num_points] - curve->y[curve->num_points-1]) / (float)(curve->x[curve->num_points] - curve->x[curve->num_points-1]);
			// the final slope is for interpolation beyond the end of the curve
			//该点下条直线斜率(设置与上一个直线相同)
			curve->slope[curve->num_points] = curve->slope[curve->num_points-1];	
		}
		curve->num_points++;
		return 1;
	}
	else
	{
		// we do not have room for the new point
		return 0;
	}
	
}


int16_t AP_CurveInt16_Size4_GetY(AP_CurveInt16_Size4_T *curve,int16_t x)
{
	uint16_t num_points = curve->num_points;
	uint16_t i;

	if(x<=curve->x[0])			//最小值限幅
		return curve->y[0];

	if(x >= curve->x[num_points-1])		//最大值限幅
		return curve->y[num_points-1];

	for(i=0;i<num_points-1;i++)
	{
		if(x >= curve->x[i] && x<= curve->x[i+1])
		{
			return curve->y[i] + (x-curve->x[i])*curve->slope[i];
		}
	}
	return x;
}

