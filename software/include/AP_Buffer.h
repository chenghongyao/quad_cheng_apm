#ifndef _AP_BUFFER_H_
#define _AP_BUFFER_H_
#include "sys.h"

#define  AP_FloatBuffer_SIZE_MAX	20
typedef struct
{
	uint8_t  head;
	uint8_t num_item;
	float buffer[AP_FloatBuffer_SIZE_MAX];
	uint8_t size;
}AP_BufferFloat;

void AP_BufferFloat_Init(AP_BufferFloat *apbuf, uint8_t size);
void AP_BufferFloat_Clear(AP_BufferFloat *apbuf);
void AP_BufferFloat_Push_Back(AP_BufferFloat *apbuf, float item);
float AP_BufferFloat_Pop_Front(AP_BufferFloat *apbuf, float item);
float AP_BufferFloat_Front(AP_BufferFloat *apbuf);
float AP_BufferFloat_Peek(AP_BufferFloat *apbuf, uint8_t position);
uint8_t AP_BufferFloat_Full(AP_BufferFloat *apbuf);
#endif
