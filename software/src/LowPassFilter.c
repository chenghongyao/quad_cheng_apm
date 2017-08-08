#include "LowPassFilter.h"




void low_pass_filter_set_fc(low_pass_fiter_t *lpf,float time_step, float cutoff_freq)
{
		float rc;
	   // avoid divide by zero and allow removing filtering
    if (cutoff_freq <= 0.0f) 
		{
        lpf->alpha = 1.0f;
        return;
    }

    // calculate alpha
    rc = 1/(2*MY_PI*cutoff_freq);
    lpf->alpha = time_step / (time_step + rc);
}

float low_pass_filter_apply(low_pass_fiter_t *lpf,float sample)
{
	
	    // initailise _base_value if required
    if( !lpf->base_value_set ) {
        lpf->base_value = sample;
        lpf->base_value_set = 1;
    }

    // do the filtering
    //_base_value = _alpha * (float)sample + (1.0 - _alpha) * _base_value;
    lpf->base_value = lpf->base_value + lpf->alpha * ((float)sample - lpf->base_value);

    // return the value.  Should be no need to check limits
    return lpf->base_value;
	
}

void low_pass_filter_reset(low_pass_fiter_t *lpf,float new_base_value)
{
	lpf->base_value = new_base_value; 
	lpf->base_value_set = 1;
}


