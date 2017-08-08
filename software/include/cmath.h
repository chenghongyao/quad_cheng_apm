#ifndef _CMATH_H
#define _CMATH_H
#include "sys.h"
#include "math.h"
#define MY_PI 3.1415926f
#define MY_ISZERO(_d)	(((_d)>-0.001f)&&((_d)<0.001f))
#define MY_POWER(_d)	((_d)*(_d))
#define ABS(x) ( (x)>0?(x):-(x) )
#define MAX(_x,_y) ((_x)>(_y)?(_x):(_y))
#define MIN(_x,_y) ((_x)<(_y)?(_x):(_y))


#define DEG2RAD(_deg)	((_deg)*(MY_PI/180.0f))
#define RAD2DEG(_rad)	((_rad)*(180.0f/MY_PI))
#define ANGLE_TO_RADIAN 0.01745329f //*0.01745 = /57.3	角度转弧度


//Single precision conversions
#define DEG_TO_RAD 0.017453292519943295769236907684886f
#define RAD_TO_DEG 57.295779513082320876798154814105f

//from ano
#define LIMIT( x,min,max ) ( (x) < (min)  ? (min) : ( (x) > (max) ? (max) : (x) ) )
#define safe_div(numerator,denominator,safe_value) ( (denominator == 0) ? (safe_value) : ((numerator)/(denominator)) )
#define my_pow(a) ((a)*(a))


////////////////////////////////////////////////////////////////////////

// acceleration due to gravity in m/s/s
#define GRAVITY_MSS 9.80665f

//矩阵
void matrix_mv(float *ovec, float *mat, float *vec, uint8_t row, uint8_t col);
void matrix_dot(float *omat, float *mat1, float *mat2, uint8_t row1, uint8_t col1, uint8_t col2);
void matrix_transpose(float *omat, float *mat, uint8_t row, uint8_t col);
void matrix_eye(float *omat, uint8_t n);
float matrix_inv3b3(float *omat, float *mat);
float matrix_det2b2(float *mat);
float matrix_det3b3(float *mat);
float matrix_det4b4(float *mat);
//向量
void vector_cross(float *ovec, float *vec1, float *vec2);
void vector_add(float *ovec, float *vec1, float *vec2, uint8_t dim);
void vector_subtract(float *ovec, float *minuend, float *subtrahend, uint8_t dim);
void vector_scale(float *ovec, float *vec, float scale, uint8_t dim);
void vector_normalized(float *vec, uint8_t dim);
//其他
void simple_3d_trans(float ref[3], float in[3], float out[3]); //小范围内正确。;
float my_sqrt(float number);
float my_invsqrt(float number);
float my_invnorm(float v[3]);
float my_norm(float v[3]);
void rad2deg(float *o, float *i, uint16_t size);
void deg2rad(float *o, float *i, uint16_t size);
float mywrap180(float x);
float my_scareroom(float x,float x_end,float deadband);
float my_deathzoom(float x,float ref,float zoom);//my_deadzone

float sq(float v);
float pythagorous2(float a, float b);
float pythagorous3(float a, float b, float c);
float radians(float deg);
float degrees(float rad);
float constrain_float(float amt, float low, float high);
int16_t constrain_int16(int16_t amt, int16_t low, int16_t high);
int32_t constrain_int32(int32_t amt, int32_t low, int32_t high);
float safe_sqrt(float v);
float safe_asin(float v);
float wrap_180_cd_float(float angle);
int32_t wrap_180_cd(int32_t error);
int32_t wrap_360_cd(int32_t error);
#endif
