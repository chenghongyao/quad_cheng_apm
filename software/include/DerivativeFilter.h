#ifndef _DERIVATIVEFILTER_H_
#define _DERIVATIVEFILTER_H_
#include "sys.h"

#define  DerivativeFilter_SIZE_MAX		20

typedef struct
{	



	uint8_t			new_data;
	float       last_slope;
	uint32_t    timestamps[DerivativeFilter_SIZE_MAX];
	float      samples[DerivativeFilter_SIZE_MAX];       // buffer of samples
	uint8_t    sample_index;               // pointer to the next empty slot in the buffer
	uint8_t		size;
}DerivativeFilter_T;

void DerivativeFilter_Init(DerivativeFilter_T *filter, uint8_t size);
void DerivativeFilter_Reset(DerivativeFilter_T *filter);
float DerivativeFilter_Apply(DerivativeFilter_T *filter, float sample);
void DerivativeFilter_Update(DerivativeFilter_T *filter, float sample, uint32_t timestamp);
float DerivativeFilter_Slope(DerivativeFilter_T *filter);




#endif
