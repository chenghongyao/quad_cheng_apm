#include "RC_Channel.h"



//////////////////////////////////////////////////////////////////////////////////////
//角度转PWM ，servo_out
static int16_t rc_range_to_pwm(rcchannel_t *rc)
{
    if (rc->high_out == rc->low_out) 
	{
        return rc->radio_trim;
    }
	//？？？？？？
    return ((long)(rc->servo_out - rc->low_out) * (long)(rc->radio_max - rc->radio_min)) / (long)(rc->high_out - rc->low_out);
}

static int16_t rc_angle_to_pwm(rcchannel_t *rc)
{
	if ((rc->servo_out * rc->reverse) > 0)
		return rc->reverse * ((long)rc->servo_out * (long)(rc->radio_max - rc->radio_trim)) / (long)rc->high;
	else
		return rc->reverse * ((long)rc->servo_out * (long)(rc->radio_trim - rc->radio_min)) / (long)rc->high;
}
//////////////////////////////////////////////////////////////////////////////////////
//PWM转角度,加死区, radio_in ->
static int16_t rc_pwm_to_range_dz(rcchannel_t *rc,uint16_t dead_zone)
{
	int16_t r_in = constrain_int16(rc->radio_in, rc->radio_min, rc->radio_max);	//限幅

	int16_t radio_trim_low;
	if(rc->reverse == -1) 			//反相
	{
		r_in = rc->radio_max - (r_in - rc->radio_min);
	}

	radio_trim_low = rc->radio_min + dead_zone;

	if (r_in > radio_trim_low)
			return (rc->low + ((long)(rc->high - rc->low) * (long)(r_in - radio_trim_low)) / (long)(rc->radio_max - radio_trim_low));
	else if (dead_zone > 0)
			return 0;
	else
			return rc->low;
}
static int16_t rc_pwm_to_range(rcchannel_t *rc)
{
	return rc_pwm_to_range_dz(rc,rc->dead_zone);
}
static int16_t rc_pwm_to_angle_dz(rcchannel_t *rc, uint16_t dead_zone)
{
	int16_t radio_trim_high = rc->radio_trim + dead_zone;		//中值以上
	int16_t radio_trim_low = rc->radio_trim - dead_zone;		//中值以下

	// prevent div by 0
	if ((radio_trim_low - rc->radio_min) == 0 || (rc->radio_max - radio_trim_high) == 0)
		return 0;

	if (rc->radio_in > radio_trim_high) //大于死区位置
	{
		return rc->reverse * ((long)rc->high * (long)(rc->radio_in - radio_trim_high)) / (long)(rc->radio_max - radio_trim_high);
	}
	else if (rc->radio_in < radio_trim_low) //小于死区位置
	{
		return rc->reverse * ((long)rc->high * (long)(rc->radio_in - radio_trim_low)) / (long)(radio_trim_low - rc->radio_min);
	}
	else
		return 0;
}

static int16_t rc_pwm_to_angle(rcchannel_t *rc)
{
	return rc_pwm_to_angle_dz(rc,rc->dead_zone);
}
///////////////////////////////////////////////////////////////////////////
void rc_calc_pwm(rcchannel_t *rc)
{
	if(rc->type == RC_CHANNEL_TYPE_RANGE) 
	{
		rc->pwm_out         = rc_range_to_pwm(rc);	
		rc->radio_out = (rc->reverse >= 0) ? (rc->radio_min + rc->pwm_out) : (rc->radio_max - rc->pwm_out);//反向处理
	}
	else if(rc->type == RC_CHANNEL_TYPE_ANGLE_RAW) 
	{
		rc->pwm_out         = (float)rc->servo_out * 0.1f;
		rc->radio_out       = (rc->pwm_out * rc->reverse) + rc->radio_trim;

	}else
	{     // RC_CHANNEL_TYPE_ANGLE	
		rc->pwm_out         = rc_angle_to_pwm(rc);
		rc->radio_out       = rc->pwm_out + rc->radio_trim;
	}

    rc->radio_out = constrain_int16(rc->radio_out, rc->radio_min, rc->radio_max);
}


void rc_set_angle(rcchannel_t *rc,int16_t angle)
{	
	rc->type = RC_CHANNEL_TYPE_ANGLE;
	rc->high = angle;				//ANGLE最大值
}

void rc_set_range(rcchannel_t *rc,int16_t low, int16_t high)
{
    rc->type           = RC_CHANNEL_TYPE_RANGE;

    rc->high           = high;//输入用
    rc->low            = low;

    rc->high_out       = high;//输出用??
    rc->low_out        = low;
}

void rc_set_type(rcchannel_t *rc,uint8_t t)
{
	rc->type = t;
}


//设置从接收机得到的值 pwm->radio_in control_in
void rc_set_pwm(rcchannel_t *rc,int16_t pwm)
{
	rc->radio_in = pwm;

	if (rc->type == RC_CHANNEL_TYPE_RANGE) 
	{
		rc->control_in = rc_pwm_to_range(rc);
	} 
	else //RC_CHANNEL_TYPE_ANGLE, RC_CHANNEL_TYPE_ANGLE_RAW
	{
		rc->control_in = rc_pwm_to_angle(rc);
	}
}
void rc_set_dead_zone(rcchannel_t *rc,int16_t dzone)
{
	rc->dead_zone = dzone;
}
