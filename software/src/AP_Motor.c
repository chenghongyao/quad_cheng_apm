#include "AP_Motor.h"
#include "ahrs_dcm.h"
#include "cmath.h"
#include "sys.h"
#include "paramter.h"
#include "define.h"

#include "AP_InertialSensor.h"
#include "AP_State.h"
#include "radio.h"
#include "notify.h"

int16_t apmotor_get_angle_boost(int16_t throttle_pwm)
{
    float temp = ahrs.cos_pitch * ahrs.cos_roll;
    int16_t throttle_out;

    temp = constrain_float(temp, 0.5f, 1.0f);
	
    // apply scale and constrain throttle
    // To-Do: move throttle_min and throttle_max into the AP_Vehicles class?
    throttle_out = constrain_float((float)(throttle_pwm-g.throttle_min) * temp + g.throttle_min, g.throttle_min, 1000);
	// record angle boost for logging
   
	apmotor.angle_boost = throttle_out - throttle_pwm;

    return throttle_out;
}



//更新最大油门，使油门缓慢启动
void apmotor_update_max_throttle()
{
		//解锁怠速，应该再解锁时设置
	if(apmotor.flags.slow_start_low_end)
	{
		apmotor.spin_when_armed_ramped += AP_MOTOR_SLOW_START_LOW_END_INCREMENT;
		if (apmotor.spin_when_armed_ramped >= apmotor.spin_when_armed) 
		{
				apmotor.spin_when_armed_ramped = apmotor.spin_when_armed;
				apmotor.flags.slow_start_low_end = 0;
		}
	}
	
	if(!apmotor.flags.slow_start)	////非慢启动，则用最大油门
	{
		return;
	}
	
	apmotor.max_throttle += AP_MOTOR_SLOW_START_INCREMENT;
	if(apmotor.max_throttle >= apmotor.rc_throttle->servo_out)
	{
		apmotor.max_throttle = AP_MOTORS_DEFAULT_MAX_THROTTLE;
		apmotor.flags.slow_start = 0;
	}
}




void apmotor_ouput_min()
{
	uint8_t i;
	apmotor.limit.roll_pitch = 1;
	apmotor.limit.yaw = 1;
	apmotor.limit.throttle_lower = 1;
	apmotor.limit.throttle_upper = 0;
	
	for( i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++ ) 
	{
		if(apmotor.motor_enabled[i])
		{
			motor_write(i,apmotor.rc_throttle->radio_min);		//写最小值
		}
   }
}


uint8_t setup_throttle_curve()
{
	int16_t min_pwm = apmotor.rc_throttle->radio_min;
	int16_t max_pwm = apmotor.rc_throttle->radio_max;
	int16_t mid_throttle_pwm = (max_pwm + min_pwm) / 2;
	int16_t mid_thrust_pwm = min_pwm + (float)(max_pwm - min_pwm) * ((float)apmotor.throttle_curve_mid / 100.0f);
	int16_t max_thrust_pwm = min_pwm + (float)(max_pwm - min_pwm) * ((float)apmotor.throttle_curve_max / 100.0f);
	uint8_t retval = 1;

	// some basic checks that the curve is valid
	if (mid_thrust_pwm >= (min_pwm + apmotor.min_throttle) && mid_thrust_pwm <= max_pwm && max_thrust_pwm >= mid_thrust_pwm && max_thrust_pwm <= max_pwm) 
	{
		// clear curve	
		// curve initialisation
		retval &= AP_CurveInt16_Size4_AddPoint(&apmotor.throttle_curve,min_pwm, min_pwm);
		retval &= AP_CurveInt16_Size4_AddPoint(&apmotor.throttle_curve,min_pwm + apmotor.min_throttle, min_pwm + apmotor.min_throttle);
		retval &= AP_CurveInt16_Size4_AddPoint(&apmotor.throttle_curve,mid_throttle_pwm, mid_thrust_pwm);
		retval &= AP_CurveInt16_Size4_AddPoint(&apmotor.throttle_curve,max_pwm, max_thrust_pwm);
		// return success
		return retval;
	}
	else
	{
		retval = 0;
	}


	// disable throttle curve if not set-up corrrectly
	if (!retval) 
	{
		apmotor.throttle_curve_enabled = 0;
	}

	return retval;
}


void apmotor_output_armed()
{
	int8_t i;
	int16_t out_min_pwm = apmotor.rc_throttle->radio_min + apmotor.min_throttle;   // 最小的pwm输出(到motor)
	int16_t out_max_pwm =  apmotor.rc_throttle->radio_max;						  // 最大的pwm输出2000?
	int16_t out_mid_pwm = (out_min_pwm+out_max_pwm)/2;                  		  // pwm输出中值 ~~1500
	
	int16_t out_best_thr_pwm;		 //最佳输出油门(悬停 the is the best throttle we can come up which provides good control without climbing
	float	rpy_scale = 1.0; 		//缩放rpy使其不超过范围this is used to scale the roll, pitch and yaw to fit within the motor limits，

	int16_t rpy_out[AP_MOTORS_MAX_NUM_MOTORS];
	int16_t motor_out[AP_MOTORS_MAX_NUM_MOTORS];
	int16_t motor_mid;

	int16_t rpy_low = 0;    // lowest motor value  ,记录最小的电机输出
	int16_t rpy_high = 0;   // highest motor value,记录最大的电机输出
	int16_t yaw_allowed;    // amount of yaw we can fit in,允许的yaw输出  
	int16_t thr_adj;        // 叠加在最佳油门的调整值the difference between the pilot's desired throttle and out_best_thr_pwm (the throttle that is actually provided)
	int16_t thr_adj_max;		//最大调整值
	int16_t thr_adj_min;
	
	//先初始化标志位
	apmotor.limit.roll_pitch = 0;		
	apmotor.limit.yaw = 0;
	apmotor.limit.throttle_lower = 0;		//输出油门比期望油门小标志
	apmotor.limit.throttle_upper = 0;		//输出油门比期望油门大标志
	
	
	if(apmotor.rc_throttle->servo_out <= 0)		//radio_min???
	{			//油门输出为0
		apmotor.rc_throttle->servo_out = 0;
		apmotor.limit.throttle_lower = 1;
	}
	
	
	if(apmotor.rc_throttle->servo_out >= apmotor.max_throttle) 		//有时会被限制住
	{//油门输出最高
		apmotor.rc_throttle->servo_out = apmotor.max_throttle;
		apmotor.limit.throttle_upper = 1;
	}
	
	// capture desired roll, pitch, yaw and throttle from receiver 
	//获取外环输出值									//servo_out    ->   radio_out,
	rc_calc_pwm(apmotor.rc_roll);		
	rc_calc_pwm(apmotor.rc_pitch);		//servo_out*0.1 = pwm_out, radio_out =  pwm_out + radio_trim
	rc_calc_pwm(apmotor.rc_yaw);
	rc_calc_pwm(apmotor.rc_throttle);
	
	if(apmotor.rc_throttle->servo_out == 0)		//0输出
	{
		//慢慢怠速转动到最小油门
		if (apmotor.spin_when_armed_ramped < 0) 
		{
				 apmotor.spin_when_armed_ramped = 0;
		}
	
		if (apmotor.spin_when_armed_ramped > apmotor.min_throttle) 		
		{
				apmotor.spin_when_armed_ramped = apmotor.min_throttle;//怠速转动有油门
		}
			
		//油门慢慢增加！
		for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) 
		{
			// spin motors at minimum
			if (apmotor.motor_enabled[i]) 
			{
				motor_out[i] = apmotor.rc_throttle->radio_min + apmotor.spin_when_armed_ramped;//一定叠加上radio_min
			}
		}
		apmotor.limit.roll_pitch = 1;
		apmotor.limit.yaw = 1;
	}
	else		//油门非0
	{
		// check if throttle is below limit
		if (apmotor.rc_throttle->servo_out <= apmotor.min_throttle) 
		{  // perhaps being at min throttle itself is not a problem, only being under is
			apmotor.limit.throttle_lower = 1;
		}
		
		//计算电机的原始roll,pitch的输出
		for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) 
		{
			if (apmotor.motor_enabled[i]) 
			{
				rpy_out[i] = apmotor.rc_roll->pwm_out * apmotor.roll_factor[i] +		
								apmotor.rc_pitch->pwm_out * apmotor.pitch_factor[i];

				// record lowest roll pitch command
				if (rpy_out[i] < rpy_low) 
				{
						rpy_low = rpy_out[i];
				}


				// record highest roll pich command
				if (rpy_out[i] > rpy_high) {
						rpy_high = rpy_out[i];
				}
			}
		}
		
		//thro.radio_out 期望油门??
		//1.油门大时，pwm中值- 电机rp量的估计值作为最佳油门,剩余的值作为yaw控制量的空间,最佳油门小于该值
		//2.油门相当小时使用
		//	a.期望油门小于悬停油门时直接使用期望油门
		//  b.期望油门大于悬停油门时,取期望油门和悬停油门的平均(也就是说油门变化速率减半)
		//当油门相当小时才选择第二种情况,最小值为悬停油门/2,当大于悬停油门时直接使用期望油门
		//当油门较大时,使用到mid_pwm的剩余量 （再out_mid_pwm附近）
		//MAX项,油门较小时直接输出,大于悬停油门时减慢速率
		//为了给yaw提供空间,
		motor_mid = (rpy_low+rpy_high)/2;		//
		out_best_thr_pwm = MIN(out_mid_pwm - motor_mid, MAX(apmotor.rc_throttle->radio_out, (apmotor.rc_throttle->radio_out+apmotor.hover_out)/2));

		// calculate amount of yaw we can fit into the throttle range
		// this is always equal to or less than the requested yaw from the pilot or rate controller,允许的yaw控制量??
		//前一项得到油门最小的可变化空间, 减去(roll,pitch)控制量的变化空间??
		yaw_allowed = MIN(out_max_pwm - out_best_thr_pwm, out_best_thr_pwm - out_min_pwm) - (rpy_high-rpy_low)/2;//(rpy_high+rpy_low)/2??
		yaw_allowed = MAX(yaw_allowed, AP_MOTORS_MATRIX_YAW_LOWER_LIMIT_PWM); //允许量不能过小,最小200

		//yaw_allowed 与 期望yaw去最小值
		if(apmotor.rc_yaw->pwm_out >= 0)
		{//向右转
				// if yawing right
			if (yaw_allowed > apmotor.rc_yaw->pwm_out) //取最小值
			{
				yaw_allowed = apmotor.rc_yaw->pwm_out; // to-do: this is bad form for yaw_allows to change meaning to become the amount that we are going to output
			}
			else
			{
				apmotor.limit.yaw = 1;
			}
		}
		else
		{
			// if yawing left
			yaw_allowed = -yaw_allowed;
			if( yaw_allowed < apmotor.rc_yaw->pwm_out ) //取最小值
			{		
				yaw_allowed = apmotor.rc_yaw->pwm_out; // to-do: this is bad form for yaw_allows to change meaning to become the amount that we are going to output
			}
			else
			{
				apmotor.limit.yaw = 1;
			}
		}
	
		//加入yaw控制量
		rpy_low = 0;
		rpy_high = 0;
		for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) 
		{
			if (apmotor.motor_enabled[i]) 
			{
				rpy_out[i] =    rpy_out[i] + yaw_allowed *apmotor.yaw_factor[i];		//添加yaw控制量
				// record lowest roll+pitch+yaw command	//再次求出输出最大最小值
				if (rpy_out[i] < rpy_low)rpy_low = rpy_out[i];
				if (rpy_out[i] > rpy_high) rpy_high = rpy_out[i];
			}
		}
		
		// check everything fits,输出会在最佳油门上加上调整值,以接近期望的油门
		thr_adj = apmotor.rc_throttle->radio_out - out_best_thr_pwm;			//期望油门与最佳油门的差值
		thr_adj_max = MAX(out_max_pwm - (out_best_thr_pwm + rpy_high), 0);		//PWM剩余值>0
		// 但是为了加入调整值之后不要超过最大PWM
		// calc upper and lower limits of thr_adj
		//(所以)计算PWM最大值与解算输出的差值 
		if (thr_adj > 0) 
		{//期望油门大于最佳油门
			// increase throttle as close as possible to requested throttle
			// without going over out_max_pwm		
			if (thr_adj > thr_adj_max) // 大于最大调整值
			{
					thr_adj = thr_adj_max;	
					// we haven't even been able to apply full throttle command
					apmotor.limit.throttle_upper = 1;
			}
		}
		else if(thr_adj < 0)
		{		//以下同
			// decrease throttle as close as possible to requested throttle
			// without going under out_min_pwm or over out_max_pwm
			// earlier code ensures we can't break both boundaries
			thr_adj_min = MIN(out_min_pwm-(out_best_thr_pwm+rpy_low),0);
			if (thr_adj > thr_adj_max)	//毫无意义~~ ,此时thr_adj<0,但thr_adj_max>0
			{
					thr_adj = thr_adj_max;
					apmotor.limit.throttle_upper = 1;			
			}
			if (thr_adj < thr_adj_min) 
			{
					thr_adj = thr_adj_min;
					apmotor.limit.throttle_lower = 1;
			}
		}
		
		// do we need to reduce roll, pitch, yaw command
		// earlier code does not allow both limit's to be passed simultainiously with abs(_yaw_factor)<1
		//按比例缩放各电机roll,pitch,yaw控制量,保证输出不超范围
		if((rpy_low+out_best_thr_pwm)+thr_adj < out_min_pwm)
		{//小于输出最小值
			rpy_scale = (float)(out_min_pwm-thr_adj-out_best_thr_pwm)/rpy_low;//剩余油门/最小rpy输出, 输出扩大
			// we haven't even been able to apply full roll, pitch and minimal yaw without scaling
			apmotor.limit.roll_pitch = 1;
			apmotor.limit.yaw = 1;
		}
		else if((rpy_high+out_best_thr_pwm)+thr_adj > out_max_pwm)
		{
			rpy_scale = (float)(out_max_pwm-thr_adj-out_best_thr_pwm)/rpy_high;//剩余油门量/最大rpy输出,输出减小
			// we haven't even been able to apply full roll, pitch and minimal yaw without scaling
			apmotor.limit.roll_pitch = 1;
			apmotor.limit.yaw = 1;
		}
		
		 // add scaled roll, pitch, constrained yaw and throttle for each motor
		for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) 
		{
			if (apmotor.motor_enabled[i]) 
			{
				motor_out[i] = out_best_thr_pwm+thr_adj + rpy_scale*rpy_out[i];
			}
		}
		
		// adjust for throttle curve
		//油门曲线调整
		if (apmotor.throttle_curve_enabled) 
		{
			for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) 
			{
				if (apmotor.motor_enabled[i])
				{
					motor_out[i] = AP_CurveInt16_Size4_GetY(&apmotor.throttle_curve,motor_out[i]);	
					motor_out[i] = constrain_int16(motor_out[i], out_min_pwm, out_max_pwm);
				}
			}
		}

	}
	
	// send output to each motor可以输出了
	for( i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++ )
	{
		if(apmotor.motor_enabled[i] ) 
		{
			motor_write(i, motor_out[i]);
		}
	}

}



void apmotor_output_disarmed()
{
	apmotor_ouput_min();
}


void apmotor_output()
{
	apmotor_update_max_throttle();//!!判断解锁怠速和油门慢启动
	if(apmotor.flags.armed)
	{
		apmotor_output_armed();//!!!!??
	}
	else
	{
		apmotor_output_disarmed();		//!!上锁状态下输出最小值
	}
}



void apmotor_remove_motor(uint8_t motor_num)
{
	if (motor_num < AP_MOTORS_MAX_NUM_MOTORS)
	{
		apmotor.motor_enabled[motor_num] = 0;
		apmotor.roll_factor[motor_num] = 0;
		apmotor.pitch_factor[motor_num] = 0;
		apmotor.yaw_factor[motor_num] = 0;
	}
}


void apmotor_remove_all_motors()
{
	uint8_t i;
	for (i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++)
	{
		apmotor_remove_motor(i);
	}
}

void apmotor_add_motor_raw(uint8_t motor_num,float roll_fac,float pitch_fac,float yaw_fac)
{
	if (motor_num < AP_MOTORS_MAX_NUM_MOTORS)
	{
		apmotor.motor_enabled[motor_num] = 1;
		apmotor.roll_factor[motor_num] = roll_fac;
		apmotor.pitch_factor[motor_num] = pitch_fac;
		apmotor.yaw_factor[motor_num] = yaw_fac;
	}
}

void apmotor_add_motor(uint8_t motor_num, float angle_degree, float yaw_fac)
{
	apmotor_add_motor_raw(motor_num, cosf(radians(angle_degree + 90)), cosf(radians(angle_degree)), yaw_fac);
}

void apmotor_init(rcchannel_t *rc_roll, rcchannel_t *rc_pitch,rcchannel_t *rc_throttle,rcchannel_t *rc_yaw)
{
	//参数初始化
	apmotor.rc_roll = rc_roll;
	apmotor.rc_pitch = rc_pitch;
	apmotor.rc_yaw = rc_yaw;
	apmotor.rc_throttle = rc_throttle;

	apmotor.min_throttle = AP_MOTORS_DEFAULT_MIN_THROTTLE;		//throttle的范围在0-1000
	apmotor.max_throttle = AP_MOTORS_DEFAULT_MAX_THROTTLE;

	apmotor.hover_out	 = AP_MOTORS_DEFAULT_MID_THROTTLE;		//!!!!!一定要重新设置,该值要叠加radio_min


	apmotor.flags.armed = 0;
	apmotor.spin_when_armed = 0;
	apmotor.flags.slow_start = 0;
	apmotor.flags.slow_start_low_end = 0;			//油门缓慢启动
	apmotor.spin_when_armed = AP_MOTORS_SPIN_WHEN_ARMED;	//怠速转动油门
	

	//电机初始化
	apmotor_remove_all_motors();
	apmotor_add_motor(AP_MOTORS_MOT_1, 45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW);
	apmotor_add_motor(AP_MOTORS_MOT_2, -135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW);
	apmotor_add_motor(AP_MOTORS_MOT_3, -45, AP_MOTORS_MATRIX_YAW_FACTOR_CW);
	apmotor_add_motor(AP_MOTORS_MOT_4, 135, AP_MOTORS_MATRIX_YAW_FACTOR_CW);
	//
	apmotor.throttle_curve_enabled = THROTTLE_CURVE_ENABLED;		
	apmotor.throttle_curve_max = THROTTLE_CURVE_MAX_THRUST;		//单位:%
	apmotor.throttle_curve_mid = THROTTLE_CURVE_MID_THRUST;
	setup_throttle_curve();
}


//设置悬停油门
void apmotor_set_mid_throttle(uint16_t mid_throttle)
{
	apmotor.hover_out = apmotor.rc_throttle->radio_min + (float)(apmotor.rc_throttle->radio_max - apmotor.rc_throttle->radio_min) * mid_throttle / 1000.0f;
}

///////////////////////////////////////////////////////////
void pre_arm_rc_checks()
{
    // exit immediately if we've already successfully performed the pre-arm rc check
    if( ap.flags.pre_arm_rc_check ) 
		{
        return;
    }

    // set rc-checks to success if RC checks are disabled
    if ((g.arming_check != ARMING_CHECK_ALL) && !(g.arming_check & ARMING_CHECK_RC))
		{
        set_pre_arm_rc_check(1);
        return;
    }

//    // check if radio has been calibrated
//    if(!g.rc_3.radio_min.load() && !g.rc_3.radio_max.load()) {
//        return;
//    }

//    // check channels 1 & 2 have min <= 1300 and max >= 1700
//    if (g.rc_1.radio_min > 1300 || g.rc_1.radio_max < 1700 || g.rc_2.radio_min > 1300 || g.rc_2.radio_max < 1700) {
//        return;
//    }

//    // check channels 3 & 4 have min <= 1300 and max >= 1700
//    if (g.rc_3.radio_min > 1300 || g.rc_3.radio_max < 1700 || g.rc_4.radio_min > 1300 || g.rc_4.radio_max < 1700) {
//        return;
//    }

    // if we've gotten this far rc is ok
    set_pre_arm_rc_check(1);
}

void pre_arm_checks(uint8_t display_failure)
{
    // exit immediately if already armed
    if (apmotor.flags.armed) 	//已经解锁了，直接退出
		{		
        return;
    }
    // exit immediately if we've already successfully performed the pre-arm check
    // run gps checks because results may change and affect LED colour
    if (ap.flags.pre_arm_check)//已经检查过了
		{		
        return;
    }

    // succeed if pre arm checks are disabled
    if(g.arming_check == ARMING_CHECK_NONE) //禁止了解锁检测
		{
        set_pre_arm_check(1);				//已经检测过了
        set_pre_arm_rc_check(1);		//解锁RC检测过了
        return;
    }

    // pre-arm rc checks a prerequisite
    pre_arm_rc_checks();		//解锁RC检测
		
    if(!ap.flags.pre_arm_rc_check) //遥控器未校准
		{
        if (display_failure) 
				{
            //gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: RC not calibrated"));
        }
        return;
    }


    // check INS
    if ((g.arming_check == ARMING_CHECK_ALL) || (g.arming_check & ARMING_CHECK_INS))
		{
        // check accelerometers have been calibrated
			if(!ins.calibrated) 
			{
					if (display_failure) 
					{
						 // gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: INS not calibrated"));
					}
					return;
			}
				
			// check gyros calibrated successfully
			if(!ins.gyro_cal_ok)
			{
					if (display_failure) 
					{
				 //     gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: Gyro cal failed"));
					}
					return;
			}
    }
		


    // check various parameter values
    if ((g.arming_check == ARMING_CHECK_ALL) || (g.arming_check & ARMING_CHECK_PARAMETERS))
		{
			
//        // failsafe parameter checks
					//失控油门设置
//        if (g.failsafe_throttle) 
//				{
//            // check throttle min is above throttle failsafe trigger and that the trigger is above ppm encoder's loss-of-signal value of 900
//            if (g.rc_3.radio_min <= g.failsafe_throttle_value+10 || g.failsafe_throttle_value < 910) 
//						{
//                if (display_failure) 
//								{
//                    //gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: Check FS_THR_VALUE"));
//                }
//                return;
//            }
//        }
        // lean angle parameter check
				if (g.angle_max < 1000 || g.angle_max > 8000)
				{
            if (display_failure) 
						{
                //gcs_send_text_P(SEVERITY_HIGH,PSTR("PreArm: Check ANGLE_MAX"));
            }
            return;
        }
    }

    // if we've gotten this far then pre arm checks have completed
    set_pre_arm_check(1);
}




uint8_t arm_checks(uint8_t display_failure)
{
	// succeed if arming checks are disabled
	if (g. arming_check == ARMING_CHECK_NONE)//没有解锁检测
	{
		return 1;
	}
	
	// check parameters
	if ((g.arming_check == ARMING_CHECK_ALL) || (g.arming_check & ARMING_CHECK_PARAMETERS))
	{
		// check throttle is above failsafe throttle
		if (g.failsafe_throttle != FS_THR_DISABLED && g.rc_3.radio_in < g.failsafe_throttle_value)
		{
			return 0;
		}
	}
	
	// check lean angle
	//起飞角度过大
	if ((g.arming_check == ARMING_CHECK_ALL) || (g.arming_check & ARMING_CHECK_INS)) 
	{
		if (ABS(ahrs.roll_sensor) > g.angle_max || ABS(ahrs.pitch_sensor) > g.angle_max)
		{
			return 0;
		}
	}
	// if we've gotten this far all is ok
	return 1;
}




extern int32_t initial_armed_bearing;		//解锁时的方向,在接解锁时读取 init_arm_motors中初始化,(解锁动作调用)
extern void init_simple_bearing(void);
 uint8_t init_arm_motors()
{
	// arming marker
	// Flag used to track if we have armed the motors the first time.
	// This is used to decide if we should run the ground_start routine
	// which calibrates the IMU
	static uint8_t did_ground_start = 0;//从地面开始
	static uint8_t in_arm_motors = 0;	//进入函数标志,防止函数重入

	// exit immediately if already in this function
	if (in_arm_motors) //重复进入了
	{		
		return 0;
	}
	in_arm_motors = 1;		//


	//========关掉失控保护
	// disable cpu failsafe because initialising everything takes a while
//	failsafe_disable();			//????

	// Remember Orientation,记录方向角,无头模式用
	// --------------------
	init_simple_bearing();									
	

	
	if (did_ground_start == 0)
	{		//第一次调用
		ahrs_startup_ground(1);			//,gyro漂移设为0,ahrs快速纠正
		
		//
		did_ground_start = 1;
	}

// go back to normal AHRS gains
	ahrs.flags.fast_ground_gains = 0;		//关闭快速纠正.......

	// enable gps velocity based centrefugal force compensation
	ahrs.flags.armed = 1;				//ahrs解锁标志

	
	
	// set hover throttle,设置悬停油门apmotor.hover,默认500
	apmotor_set_mid_throttle(g.throttle_mid);


	// Cancel arming if throttle is raised too high so that copter does not suddenly take off
	read_radio();				//立刻读一下输入						

	if (g.rc_3.control_in > g.throttle_cruise && g.throttle_cruise > 100) 
	{   //油门突然变高了,上锁以防突然起飞
		apmotor_ouput_min();
		//failsafe_enable();		
		in_arm_motors = 0;
		return 0;		//解锁失败
	}

	// short delay to allow reading of rc inputs
	delay_us(5);

	// enable output to motors
	apmotor_ouput_min();			//使能输出(armed??),输出最小

	// finally actually arm the motors
	apmotor.flags.armed = 1;
	
	
	debug("arm success");
	notify.flags.armed = 1;
	// reenable failsafe
//	failsafe_enable();		//重新打开失控保护

	
	
	// flag exiting this function
	in_arm_motors = 0;		//退出函数

	// return success
	return 1;			//解锁成功
}


void init_disarm_motors()
{
	// return immediately if we are already disarmed
	if (!apmotor.flags.armed)		//已经上锁了,没在地面则不允许上锁?
	{
		return;
	}

	// we are not in the air
	set_land_complete(1);
	set_land_complete_maybe(1);

	// setup fast AHRS gains to get right attitude
	ahrs.flags.fast_ground_gains = 1;		//再地面上，快速纠正
	ahrs.flags.armed = 0;
	apmotor.flags.armed = 0;		//上锁
	notify.flags.armed = 0;
	debug("disarm success");

}
