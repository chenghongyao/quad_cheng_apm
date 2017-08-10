#include "DerivativeFilter.h"

void DerivativeFilter_Init(DerivativeFilter_T *filter, uint8_t size)
{
	filter->size = size;
	DerivativeFilter_Reset(filter);

}
void DerivativeFilter_Reset(DerivativeFilter_T *filter)
{
	int8_t i;
	// clear samples buffer
	for (i = 0; i < filter->size; i++) 
	{
		filter->samples[i] = 0;
	}
	// reset index back to beginning of the array
	filter->sample_index = 0;
}


float DerivativeFilter_Apply(DerivativeFilter_T *filter, float sample)
{
	// add sample to array
	filter->samples[filter->sample_index++] = sample;
		
	// wrap index if necessary
	if (filter->sample_index >= filter->size)
		filter->sample_index = 0;

	// base class doesn't know what filtering to do so we just return the raw sample
	return sample;
}
void DerivativeFilter_Update(DerivativeFilter_T *filter, float sample, uint32_t timestamp)
{
	uint8_t i = filter->sample_index;
	uint8_t i1;
	//上一个数据索引
	if (i == 0)
	{
		i1 = filter->size - 1;
	}
	else 
	{
		i1 = i - 1;
	}
	if(filter->timestamps[i1] == timestamp)
	{
		// this is not a new timestamp - ignore
		return;
	}
	// add timestamp before we apply to FilterWithBuffer
	filter->timestamps[i] = timestamp;
	// call parent's apply function to get the sample into the array
	DerivativeFilter_Apply(filter,	sample);	
	filter->new_data = 1;
}

float DerivativeFilter_Slope(DerivativeFilter_T *filter)
{
	
	float result = 0;
	if (!filter->new_data)
	{
		//printf("no new data\n");
		return filter->last_slope;
	}


	// use f() to make the code match the maths a bit better. Note
	// that unlike an average filter, we care about the order of the elements
#define f(i) filter->samples[(((filter->sample_index-1)+i+1)+3*filter->size/2) % filter->size]
#define x(i) filter->timestamps[(((filter->sample_index-1)+i+1)+3*filter->size/2) % filter->size]

	if (filter->timestamps[filter->size - 1] == filter->timestamps[filter->size - 2])
	{
		// we haven't filled the buffer yet - assume zero derivative
		return 0;
	}

	// N in the paper is FILTER_SIZE
	switch (filter->size)
	{
	case 5:
		result = 2 * 2 * (f(1) - f(-1)) / (x(1) - x(-1))+ 4 * 1 * (f(2) - f(-2)) / (x(2) - x(-2));
		result /= 8;
		break;
	case 7:
		
		result = 2 * 5 * (f(1) - f(-1)) / (x(1) - x(-1)) + 4 * 4 * (f(2) - f(-2)) / (x(2) - x(-2))+ 6 * 1 * (f(3) - f(-3)) / (x(3) - x(-3));
		result /= 32;
		break;
	case 9:
		result = 2 * 14 * (f(1) - f(-1)) / (x(1) - x(-1))
			+ 4 * 14 * (f(2) - f(-2)) / (x(2) - x(-2))
			+ 6 * 6 * (f(3) - f(-3)) / (x(3) - x(-3))
			+ 8 * 1 * (f(4) - f(-4)) / (x(4) - x(-4));
		result /= 128;
		break;
	case 11:
		result = 2 * 42 * (f(1) - f(-1)) / (x(1) - x(-1))
			+ 4 * 48 * (f(2) - f(-2)) / (x(2) - x(-2))
			+ 6 * 27 * (f(3) - f(-3)) / (x(3) - x(-3))
			+ 8 * 8 * (f(4) - f(-4)) / (x(4) - x(-4))
			+ 10 * 1 * (f(5) - f(-5)) / (x(5) - x(-5));
		result /= 512;
		break;
	default:
		result = 0;
		break;
	}

	// cope with numerical errors
	if (isnan(result) || isinf(result)) 
	{
		result = 0;
	}

	
	filter->new_data = 0;
	filter->last_slope = result;

	return result;
}
