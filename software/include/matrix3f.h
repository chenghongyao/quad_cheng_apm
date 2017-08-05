#ifndef _MATRIX3F_H_
#define _MATRIX3F_H_
#include "vector3f.h"
#include "vector2f.h"
#include "sys.h"


typedef struct
{
	vector3f_t a;	
	vector3f_t b;
	vector3f_t c;
}matrix3f_t;

void matrix3f_copy(matrix3f_t *sou,matrix3f_t *dst);
void matrix3f_rotate(matrix3f_t *mat,vector3f_t *g);
void matrix3f_rotateXYinv(matrix3f_t *mat,vector3f_t *g);

void matrix3f_normalization(matrix3f_t *mat);

void matrix3f_transport(matrix3f_t *mat,matrix3f_t *out);
void matrix3f_add(matrix3f_t *dst,matrix3f_t *sou,matrix3f_t *out);
void matrix3f_minus(matrix3f_t *dst,matrix3f_t *sou,matrix3f_t *out);
void matrix3f_mul(matrix3f_t *mat,vector3f_t *vec,vector3f_t *out);
void matrix3f_mul_transpose(matrix3f_t *mat,vector3f_t *vec,vector3f_t *out);
void matrix3f_mulXY(matrix3f_t *mat, vector3f_t *vec, vector2f_t *out);

uint8_t matrix3f_isnan(matrix3f_t *mat);
void matrix3f_zero(matrix3f_t *mat);
void matrix3f_identify(matrix3f_t *mat);
void matrix3f_to_euler(matrix3f_t *mat,float *roll,float *pitch,float *yaw);
void matrix3f_from_euler(matrix3f_t *mat,float roll,float pitch,float yaw);
	
#endif

