#ifndef _MAT_H
#define _MAT_H
#include "sys.h"


enum
{
	MAT_DS_ACCEL = 1,
	MAT_DS_GYRO,
	MAT_DS_STATUS,
	MAT_DS_ELLIPSOID,
	MAT_DS_LPF,
	MAT_DS_NOISE,
	MAT_DS_THROFIT
};





#define MAT_SENDDATA(_datbuf,_datlen) nrf24l01_sendAckPacket(_datbuf, _datlen);
//#define MAT_SENDDATA(_datbuf,_datlen) myputbuf(_datbuf, _datlen);


void mat_sendAccel(float ax, float ay, float az);
void mat_sendGyro(float gx, float gy, float gz);
void mat_sendStatus(float roll, float pitch, float yaw);


void mat_sendEllipsoid(float x,float y,float z);
void mat_sendLPF(float raw,float lpf);
void mat_sendNoise(float dat);

#endif
