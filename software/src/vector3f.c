#include "vector3f.h"
#include "sys.h"
#include "cmath.h"

#include "stdarg.h"
void vector3f_copy(const vector3f_t *sou,vector3f_t *dst)
{
	dst->x = sou->x;
	dst->y = sou->y;
	dst->z = sou->z;
}



//!
vector3f_t *vector3f_add(vector3f_t *dst,const vector3f_t *vec,vector3f_t *out)
{
	if(out == NULL)out = dst;
	
	out->x = dst->x + vec->x;
	out->y = dst->y + vec->y;
	out->z = dst->z + vec->z;
	return out;
}

//!
vector3f_t *vector3f_minus(vector3f_t *dst,const vector3f_t *vec,vector3f_t *out)
{
	if(out == NULL)out = dst;
	
	out->x = dst->x - vec->x;
	out->y = dst->y - vec->y;
	out->z = dst->z - vec->z;
	return out;
}


//!
float vector3f_dot(vector3f_t *vec1,vector3f_t *vec2)
{
	return ((vec1->x)*(vec2->x)+(vec1->y)*(vec2->y)+(vec1->z)*(vec2->z));
}


void vector3f_cross(vector3f_t *lvec,vector3f_t *rvec,vector3f_t *out)
{
	out->x = (lvec->y)*(rvec->z) - (lvec->z)*(rvec->y);
	out->y = (lvec->z)*(rvec->x) - (lvec->x)*(rvec->z);
	out->z = (lvec->x)*(rvec->y) - (lvec->y)*(rvec->x);
}



uint8_t vector3f_isnan(vector3f_t *vec)
{
	return (isnan(vec->x) || isnan(vec->y) || isnan(vec->z));
}
uint8_t vector3f_iszero(vector3f_t *vec)
{
	return ((vec->x == 0.0f) || (vec->y == 0.0f) || (vec->z == 0.0f));
}
uint8_t vector3f_isinf(vector3f_t *vec)
{
	return (isinf(vec->x) || isinf(vec->y) || isinf(vec->z));
}
void vector3f_zero(vector3f_t *vec)
{
	vec->x = 0.0f;
	vec->y = 0.0f;
	vec->z = 0.0f;
}

void vector3f_identify(vector3f_t *vec)
{
	vec->x = 1.0f;
	vec->y = 1.0f;
	vec->z = 1.0f;
}
void vector3f_set(vector3f_t *vec,float x,float y,float z)
{
	vec->x = x;
	vec->y = y;
	vec->z =z;
}

#define HALF_SQRT_2 0.70710678118654757f
void vector3f_rotate(vector3f_t *vec,enum Rotation rotation)
{
	float tmp;
	switch (rotation) 
	{
		case ROTATION_NONE:
		case ROTATION_MAX:
				return;
		case ROTATION_YAW_45: 
		{
				tmp = HALF_SQRT_2*(vec->x - vec->y);
				vec->y   = HALF_SQRT_2*(vec->x + vec->y);
				vec->x = tmp;
				return;
		}
		case ROTATION_YAW_90: 
		{
				tmp = vec->x; vec->x = -vec->y; vec->y = tmp;
				return;
		}
		case ROTATION_YAW_135: 
		{
				tmp = -HALF_SQRT_2*(vec->x + vec->y);
				vec->y   =  HALF_SQRT_2*(vec->x - vec->y);
				vec->x = tmp;
				return;
		}
		case ROTATION_YAW_180:
				vec->x = -vec->x; vec->y = -vec->y;
				return;
		case ROTATION_YAW_225: 
		{
				tmp = HALF_SQRT_2*(vec->y - vec->x);
				vec->y   = -HALF_SQRT_2*(vec->x + vec->y);
				vec->x = tmp;
				return;
		}
		case ROTATION_YAW_270: 
		{
				tmp = vec->x; vec->x = vec->y; vec->y = -tmp;
				return;
		}
		case ROTATION_YAW_315: 
		{
				tmp = HALF_SQRT_2*(vec->x + vec->y);
				vec->y   = HALF_SQRT_2*(vec->y - vec->x);
				vec->x = tmp;
				return;
		}
		case ROTATION_ROLL_180: 
		{
				vec->y = -vec->y; vec->z = -vec->z;
				return;
		}
		case ROTATION_ROLL_180_YAW_45: 
		{
				tmp = HALF_SQRT_2*(vec->x + vec->y);
				vec->y   = HALF_SQRT_2*(vec->x - vec->y);
				vec->x = tmp; vec->z = -vec->z;
				return;
		}
		case ROTATION_ROLL_180_YAW_90: 
		{
				tmp = vec->x; vec->x = vec->y; vec->y = tmp; vec->z = -vec->z;
				return;
		}
		case ROTATION_ROLL_180_YAW_135: 
		{
				tmp = HALF_SQRT_2*(vec->y - vec->x);
				vec->y   = HALF_SQRT_2*(vec->y + vec->x);
				vec->x = tmp; vec->z = -vec->z;
				return;
		}
		case ROTATION_PITCH_180: 
		{
				vec->x = -vec->x; vec->z = -vec->z;
				return;
		}
		case ROTATION_ROLL_180_YAW_225: 
		{
				tmp = -HALF_SQRT_2*(vec->x + vec->y);
				vec->y   =  HALF_SQRT_2*(vec->y - vec->x);
				vec->x = tmp; vec->z = -vec->z;
				return;
		}
		case ROTATION_ROLL_180_YAW_270: 
		{
				tmp = vec->x; vec->x = -vec->y; vec->y = -tmp; vec->z = -vec->z;
				return;
		}
		case ROTATION_ROLL_180_YAW_315: 
		{
				tmp =  HALF_SQRT_2*(vec->x - vec->y);
				vec->y   = -HALF_SQRT_2*(vec->x + vec->y);
				vec->x = tmp; vec->z = -vec->z;
				return;
		}
		case ROTATION_ROLL_90: 
		{
				tmp = vec->z; vec->z = vec->y; vec->y = -tmp;
				return;
		}
		case ROTATION_ROLL_90_YAW_45: 
		{
				tmp = vec->z; vec->z = vec->y; vec->y = -tmp;
				tmp = HALF_SQRT_2*(vec->x - vec->y);
				vec->y   = HALF_SQRT_2*(vec->x + vec->y);
				vec->x = tmp;
				return;
		}
		case ROTATION_ROLL_90_YAW_90: 
		{
				tmp = vec->z; vec->z = vec->y; vec->y = -tmp;
				tmp = vec->x; vec->x = -vec->y; vec->y = tmp;
				return;
		}
		case ROTATION_ROLL_90_YAW_135: 
		{
				tmp = vec->z; vec->z = vec->y; vec->y = -tmp;
				tmp = -HALF_SQRT_2*(vec->x + vec->y);
				vec->y   =  HALF_SQRT_2*(vec->x - vec->y);
				vec->x = tmp;
				return;
		}
		case ROTATION_ROLL_270: 
		{
				tmp = vec->z; vec->z = -vec->y; vec->y = tmp;
				return;
		}
		case ROTATION_ROLL_270_YAW_45: 
		{
				tmp = vec->z; vec->z = -vec->y; vec->y = tmp;
				tmp = HALF_SQRT_2*(vec->x - vec->y);
				vec->y   = HALF_SQRT_2*(vec->x + vec->y);
				vec->x = tmp;
				return;
		}
		case ROTATION_ROLL_270_YAW_90: 
		{
				tmp = vec->z; vec->z = -vec->y; vec->y = tmp;
				tmp = vec->x; vec->x = -vec->y; vec->y = tmp;
				return;
		}
		case ROTATION_ROLL_270_YAW_135: 
		{
				tmp = vec->z; vec->z = -vec->y; vec->y = tmp;
				tmp = -HALF_SQRT_2*(vec->x + vec->y);
				vec->y   =  HALF_SQRT_2*(vec->x - vec->y);
				vec->x = tmp;
				return;
		}
		case ROTATION_PITCH_90: 
		{
				tmp = vec->z; vec->z = -vec->x; vec->x = tmp;
				return;
		}
		case ROTATION_PITCH_270: 
		{
				tmp = vec->z; vec->z = vec->x; vec->x = -tmp;
				return;
		}
		case ROTATION_PITCH_180_YAW_90: 
		{
				vec->z = -vec->z;
				tmp = -vec->x; vec->x = -vec->y; vec->y = tmp;
				return;
		}
		case ROTATION_PITCH_180_YAW_270: 
		{
				vec->x = -vec->x; vec->z = -vec->z;
				tmp = vec->x; vec->x = vec->y; vec->y = -tmp;
				return;
		}
		case ROTATION_ROLL_90_PITCH_90: 
		{
				tmp = vec->z; vec->z = vec->y; vec->y = -tmp;
				tmp = vec->z; vec->z = -vec->x; vec->x = tmp;
				return;
		}
		case ROTATION_ROLL_180_PITCH_90: 
		{
				vec->y = -vec->y; vec->z = -vec->z;
				tmp = vec->z; vec->z = -vec->x; vec->x = tmp;
				return;
		}
		case ROTATION_ROLL_270_PITCH_90: 
		{
				tmp = vec->z; vec->z = -vec->y; vec->y = tmp;
				tmp = vec->z; vec->z = -vec->x; vec->x = tmp;
				return;
		}
		case ROTATION_ROLL_90_PITCH_180: 
		{
				tmp = vec->z; vec->z = vec->y; vec->y = -tmp;
				vec->x = -vec->x; vec->z = -vec->z;
				return;
		}
		case ROTATION_ROLL_270_PITCH_180: 
		{
				tmp = vec->z; vec->z = -vec->y; vec->y = tmp;
				vec->x = -vec->x; vec->z = -vec->z;
				return;
		}
		case ROTATION_ROLL_90_PITCH_270: 
		{
				tmp = vec->z; vec->z = vec->y; vec->y = -tmp;
				tmp = vec->z; vec->z = vec->x; vec->x = -tmp;
				return;
		}
		case ROTATION_ROLL_180_PITCH_270: 
		{
				vec->y = -vec->y; vec->z = -vec->z;
				tmp = vec->z; vec->z = vec->x; vec->x = -tmp;
				return;
		}
		case ROTATION_ROLL_270_PITCH_270: 
		{
				tmp = vec->z; vec->z = -vec->y; vec->y = tmp;
				tmp = vec->z; vec->z = vec->x; vec->x = -tmp;
				return;
		}
		case ROTATION_ROLL_90_PITCH_180_YAW_90: 
		{
				tmp = vec->z; vec->z = vec->y; vec->y = -tmp;
				vec->x = -vec->x; vec->z = -vec->z;
				tmp = vec->x; vec->x = -vec->y; vec->y = tmp;
				return;
		}
		case ROTATION_ROLL_90_YAW_270: 
		{
				tmp = vec->z; vec->z = vec->y; vec->y = -tmp;
				tmp = vec->x; vec->x = vec->y; vec->y = -tmp;
				return;
		}
		case ROTATION_YAW_293_PITCH_68_ROLL_90: 
		{
				float tmpx = vec->x;
				float tmpy = vec->y;
				float tmpz = vec->z;
				vec->x =  0.143039f * tmpx +  0.368776f * tmpy + -0.918446f * tmpz;
				vec->y = -0.332133f * tmpx + -0.856289f * tmpy + -0.395546f * tmpz;
				vec->z = -0.932324f * tmpx +  0.361625f * tmpy +  0.000000f * tmpz;
				return;
		}
	}
}


void vector3f_scale(vector3f_t *vec,float scale,vector3f_t *out)
{
	if(out == NULL)out = vec;
	out->x = vec->x * scale;
	out->y = vec->y * scale;
	out->z = vec->z * scale;
}
float vector3f_length(vector3f_t *vec)
{
		return my_sqrt((vec->x)*(vec->x) + (vec->y)*(vec->y) + (vec->z)*(vec->z));
}

void vector3f_normalize(vector3f_t *vec)
{
		vector3f_scale(vec,1.0f/vector3f_length(vec),NULL);
}



