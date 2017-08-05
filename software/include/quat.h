#ifndef _QUAT_H
#define _QUAT_H
#include "sys.h"


typedef struct
{
	float q[4];

	float q0q0;
	float q0q1;
	float q0q2;
	float q0q3;
	float q1q1;
	float q1q2;
	float q1q3;
	float q2q2;
	float q2q3;
	float q3q3;
}quaternion;


/*互补滤波参数*/
#define QUAT_COMP_KP	1.3f
#define QUAT_COMP_KI	0.01f
#define QUAT_INT_MAX	10.0f

void quat_normalized(float oq[4], float q[4]);
void quat_reset(quaternion *q);
void quat_prepare(quaternion *q);
void quat_initByEuler(quaternion *q,float roll,float pitch,float yaw);
void quat_update_1st(quaternion *q,float gx,float gy,float gz,float dt);
void quat_update_2nd(quaternion *q, float gx,float gy,float gz, float dt);
void quat_update_3rd(quaternion *q, float *gyro, float dt);
void quat_update_ekf(quaternion *q, float gx, float gy, float gz, float ax, float ay, float az, float dt);
void quat_update_complementy(quaternion *q, float gx, float gy, float gz, float ax, float ay, float az, float dt);
#endif
