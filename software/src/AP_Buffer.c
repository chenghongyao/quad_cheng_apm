#include "AP_Buffer.h"



void AP_BufferFloat_Init(AP_BufferFloat *apbuf,uint8_t size)
{
	apbuf->head = 0;
	apbuf->num_item = 0;
	apbuf->size = size;
}

void AP_BufferFloat_Clear(AP_BufferFloat *apbuf)
{
	apbuf->head = 0;
	apbuf->num_item = 0;
}



void AP_BufferFloat_Push_Back(AP_BufferFloat *apbuf, float item)
{
	uint8_t tail = apbuf->head + apbuf->num_item;
	
	if (tail >= apbuf->size)
	{
		tail -= apbuf->size;
	}
	apbuf->buffer[tail] = item;//加入新数据

	// increment number of items
	if (apbuf->num_item < apbuf->size)
	{
		apbuf->num_item++;
	}
	else//数据溢出,丢掉最旧的数据
	{
		// no room for new items so drop oldest item
		apbuf->head++;
		if (apbuf->head >= apbuf->size)
		{
			apbuf->head = 0;
		}
	}
}


float AP_BufferFloat_Pop_Front(AP_BufferFloat *apbuf, float item)
{	
	float result;
	// return zero if buffer is empty
	if (apbuf->num_item == 0) {
		return 0;
	}
	// get next value in buffer
	result = apbuf->buffer[apbuf->head];

	// increment to next point
	apbuf->head++;
	if (apbuf->head >= apbuf->size)
		apbuf->head = 0;

	// reduce number of items
	apbuf->num_item--;
	// return item
	return result;
}


float AP_BufferFloat_Peek(AP_BufferFloat *apbuf, uint8_t position)
{
	uint8_t j = apbuf->head + position;
	// return zero if position is out of range
	if (position >= apbuf->num_item)
	{
		return 0;
	}

	// wrap around if necessary
	if (j >= apbuf->size)
		j -= apbuf->size;

	// return desired value
	return apbuf->buffer[j];

}
float AP_BufferFloat_Front(AP_BufferFloat *apbuf)
{
	return AP_BufferFloat_Peek(apbuf,0);
}
uint8_t AP_BufferFloat_Full(AP_BufferFloat *apbuf)
{
	return (apbuf->num_item >= apbuf->size);
}

