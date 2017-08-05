#include "Compass.h"
#include "sys.h"


uint8_t Compass_read(void)
{

	return 0;
}

float Compass_calculate_heading(const matrix3f_t *dcm_mat)
{
	float heading;
	float cos_pitch_sq = 1.0f - (dcm_mat->c.x*dcm_mat->c.x);//c.x = -sθ
	float headY = compass.field.y*dcm_mat->c.z - compass.field.z*dcm_mat->c.y;
	float headX = compass.field.x*cos_pitch_sq - dcm_mat->c.x*(compass.field.y*dcm_mat->c.y + compass.field.z*dcm_mat->c.z);

	heading = constrain_float(atan2f(-headY, headX), -3.15f, 3.15f);

	// Declination correction (if supplied)
	if (fabsf(compass.declination) > 0.0f) //磁偏角？？
	{
		heading = heading + compass.declination;

		if (heading > MY_PI)    // Angle normalization (-180 deg, 180 deg)
			heading -= (2.0f * MY_PI);
		else if (heading < -MY_PI)
			heading += (2.0f * MY_PI);
	}
	return heading;
}

void Compass_init(void)
{
	compass.initialised = 0;
	vector3f_set(&compass.field, 0.0f, 0.0f, 0.0f);
	compass.last_update_time = 0;

	//TODO:系统参数
	vector3f_set(&compass.scale, 1.0f, 1.0f, 1.0f);
	vector3f_set(&compass.offset, 0.0f, 0.0f, 0.0f);
	compass.declination = 0.0f;

}




