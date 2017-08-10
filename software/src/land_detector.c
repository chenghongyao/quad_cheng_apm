#include "land_detector.h"
#include "paramter.h"
#include "AP_InertialSensor.h"
#include "InertialNav.h"
#include "PosControl.h"
#include "AP_Motor.h"
#include "ahrs_dcm.h"
#include "AP_State.h"
#include "cmath.h"
#include "Attitude.h"


// counter to verify landings
static uint16_t land_detector = LAND_DETECTOR_TRIGGER;  // we assume we are landed



uint8_t land_complete_maybe(void)
{
    return (ap.flags.land_complete || ap.flags.land_complete_maybe);
}



extern float baro_climbrate;
void update_land_detector(void)
{
	uint8_t climb_rate_low = (ABS(inav.velocity.z) < LAND_DETECTOR_CLIMBRATE_MAX) && (ABS(baro_climbrate) < LAND_DETECTOR_BARO_CLIMBRATE_MAX);
	uint8_t target_climb_rate_low = (pos_ctrl.vel_desired.z <= LAND_DETECTOR_DESIRED_CLIMBRATE_MAX);//��������С��,ҡ����������λ����
	uint8_t motor_at_lower_limit = apmotor.limit.throttle_lower;		//���Ŵﵽ����
	uint8_t throttle_low = (apmotor.rc_throttle->servo_out < get_non_takeoff_throttle());//�������С�ڷ��������(mid/2??)
	uint8_t not_rotating_fast = (vector3f_length(&ahrs.omega) < LAND_DETECTOR_ROTATION_MAX);		//��ת�ٶ�С����������ת�ٶ�30deg/sec

	if (climb_rate_low && target_climb_rate_low && motor_at_lower_limit && throttle_low && not_rotating_fast) 
	{//ȷ�Ͻ�����
		if (!ap.flags.land_complete) 
		{//�����ظ�����
			// increase counter until we hit the trigger then set land complete flag
			if (land_detector < LAND_DETECTOR_TRIGGER) 
			{		//��������һ��ʱ��
				land_detector++;
			}
			else
			{//ȷ��������
				set_land_complete(1);//���ý����־
				land_detector = LAND_DETECTOR_TRIGGER;
			}
		}
	}
	else
	{
		// we've sensed movement up or down so reset land_detector
		land_detector = 0;
		// if throttle output is high then clear landing flag
		if (apmotor.rc_throttle->servo_out > get_non_takeoff_throttle())
		{//���Ŵ����������
			set_land_complete(0);		//����û�н����־
		}
	}

	// set land maybe flag
	set_land_complete_maybe(land_detector >= LAND_DETECTOR_MAYBE_TRIGGER);		//����Ҫ����
}
