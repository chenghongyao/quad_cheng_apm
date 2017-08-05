#ifndef _VECTOR3F_H_
#define _VECTOR3F_H_

#include "Rotations.h"
#include "sys.h"
typedef struct
{
	float x;
	float y;
	float z;
	

}vector3f_t;

void vector3f_copy(const vector3f_t *sou,vector3f_t *dst);
void vector3f_rotate(vector3f_t *vec,enum Rotation rt);

vector3f_t *vector3f_add(vector3f_t *dst,const vector3f_t *vec,vector3f_t *out);
vector3f_t *vector3f_minus(vector3f_t *dst,const vector3f_t *vec,vector3f_t *out);
float vector3f_dot(vector3f_t *vec1,vector3f_t *vec2);
uint8_t vector3f_isnan(vector3f_t *vec);
uint8_t vector3f_iszero(vector3f_t *vec);
uint8_t vector3f_isinf(vector3f_t *vec);
void vector3f_zero(vector3f_t *vec);
void vector3f_scale(vector3f_t *vec,float scale,vector3f_t *out);
void vector3f_set(vector3f_t *vec,float x,float y,float z);
void vector3f_cross(vector3f_t *lvec,vector3f_t *rvec,vector3f_t *out);
float vector3f_length(vector3f_t *vec);
void vector3f_normalize(vector3f_t *vec);
#endif

