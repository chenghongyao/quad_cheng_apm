#ifndef _VECTOR2F_H_
#define _VECTOR2F_H_
#include "sys.h"
typedef struct
{
	float x;
	float y;
}vector2f_t;


void vector2f_normalize(vector2f_t *vec);
float vector2f_length(vector2f_t *vec);
void vector2f_scale(vector2f_t *vec,float scale,vector2f_t *out);
uint8_t vector2f_isinf(vector2f_t *vec);
uint8_t vector2f_isnan(vector2f_t *vec);

#endif
