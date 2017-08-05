#ifndef _AC_PID_H_
#define _AC_PID_H_




#define AC_PID_D_TERM_FILTER 0.556864f    // Default 100Hz Filter Rate with 20Hz Cutoff Frequency


typedef struct
{
	//
	float kp;
	float ki;
	float kd;
	float imax;
	//
	float integrator;
	float d_lpf_alpha;
	float last_error;
	float last_derivative;
}acpid_t;

float acpid_get_p(acpid_t *pid,float error);;
float acpid_get_i(acpid_t *pid,float error, float dt);
float acpid_get_d(acpid_t *pid,float error, float dt);
float acpid_get_integrator(acpid_t *pid);

void acpid_reset(acpid_t *pid);
void acpid_set(acpid_t *pid,float kp,float ki,float kd,float imax);

#endif



