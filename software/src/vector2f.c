#include "vector2f.h"
#include "stdio.h"
#include "cmath.h"



void vector2f_scale(vector2f_t *vec,float scale,vector2f_t *out)
{
	if(out == NULL)out = vec;
	vec->x *= scale;
	vec->y *= scale;
}
float vector2f_length(vector2f_t *vec)
{
		return my_sqrt((vec->x)*(vec->x) + (vec->y)*(vec->y));
}

void vector2f_normalize(vector2f_t *vec)
{
		vector2f_scale(vec,1.0f/vector2f_length(vec),NULL);
}

uint8_t vector2f_isnan(vector2f_t *vec)
{
	return (isinf(vec->x) || isinf(vec->y));
}
uint8_t vector2f_isinf(vector2f_t *vec)
{
	return (isinf(vec->x) || isinf(vec->y));
}
