#include "AC_PID.h"
#include "cmath.h"



float acpid_get_p(acpid_t *pid,float error)
{
	return error * pid->kp;
}

float acpid_get_i(acpid_t *pid,float error, float dt)
{
	if((pid->ki != 0) && (dt != 0)) 
	{
		pid->integrator += (error * pid->ki) * dt;
		if (pid->integrator < -pid->imax) 
		{
				pid->integrator = -pid->imax;
		} 
		else if (pid->integrator > pid->imax) 
		{
				pid->integrator = pid->imax;
		}
		return pid->integrator;
	}
  return 0;
}

float acpid_get_integrator(acpid_t *pid)
{
	return pid->integrator;
}


float acpid_get_d(acpid_t *pid,float error, float dt)
{
	if ((pid->kd != 0) && (dt != 0)) 
	{
		float derivative;
		if (isnan(pid->last_derivative)) 
		{
			// we've just done a reset, suppress the first derivative
			// term as we don't want a sudden change in input to cause
			// a large D output change			
			derivative = 0;
			pid->last_derivative = 0;
		} 
		else 
		{
			// calculate instantaneous derivative
			derivative = (error - pid->last_error) / dt;
		}

		// discrete low pass filter, cuts out the
		// high frequency noise that can drive the controller crazy,微分项滤波
		derivative = pid->last_derivative + pid->d_lpf_alpha * (derivative - pid->last_derivative);		

		// update state
		pid->last_error         = error;
		pid->last_derivative    = derivative;

		// add in derivative component
		return pid->kd * derivative;
	}
	return 0;
}

void acpid_set(acpid_t *pid, float kp, float ki, float kd, float imax)
{
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
	pid->imax = fabs(imax);

	pid->integrator = 0;
	pid->last_error = 0;
	pid->last_derivative = 0;
	pid->d_lpf_alpha = AC_PID_D_TERM_FILTER;		//fs = 100Hz , fc = 20Hz
}

void acpid_set_d_alpha(acpid_t *pid,int16_t cutoff_frequency, float time_step)
{
	  // calculate alpha
    float rc = 1/(2*MY_PI*cutoff_frequency);
    pid->d_lpf_alpha = time_step / (time_step + rc);
}
