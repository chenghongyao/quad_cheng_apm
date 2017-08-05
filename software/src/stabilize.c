#include "stabilize.h"


#include "AP_Motor.h"
#include "paramter.h"
#include "AttitudeControl.h"

extern float simple_cos_yaw;			// init_arm_motors中初始化为当前yaw(解锁动作调用)
extern float simple_sin_yaw;				

extern int32_t super_simple_last_bearing;	// init_arm_motors中初始化为当前yaw+180度(解锁动作调用)
extern float super_simple_cos_yaw;
extern float super_simple_sin_yaw;

//


void stabilize_run()
{
	int16_t target_roll, target_pitch;
	float target_yaw_rate;
	int16_t pilot_throttle_scaled;
	// if not armed or throttle at zero, set throttle to zero and exit immediately
	if(!apmotor.flags.armed || g.rc_3.control_in <= 0)	//上锁或解锁或没有油门
	{
		relax_bf_rate_controller();						//将期望速率设置为当前速率->误差为0
		set_yaw_target_to_current_heading();	//当前期望方向设为当前方向,_angle_ef_target.z
		set_throttle_out(0, 0);							//rc_throttle->servo_out = 0;??
		return;
	}
	
	update_simple_mode();						//修改roll,pitch的control_in,自稳模式下不会进入 simple_mode = 0;
	//输入量归一化处理,得到期望角度
	get_pilot_desired_lean_angles(g.rc_1.control_in, g.rc_2.control_in, &target_roll, &target_pitch);	
	//期望方向速度为直接乘一个系数(默认4.5)
	target_yaw_rate = get_pilot_desired_yaw_rate(g.rc_4.control_in);//最大值为45度/s,*4.5=202	
	//期望油门,死区处理?
//	printf("con_in=%d\n",g.rc_3.control_in);
	pilot_throttle_scaled = get_pilot_desired_throttle(g.rc_3.control_in);	//？？？？乱。。。	
	printf("con_out=%d\n",pilot_throttle_scaled);
	
	//平滑增益? 2-12,越大反应越快 目标值->rate_bf_target
	angle_ef_roll_pitch_rate_ef_yaw_smooth(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());//gain默认为12
	//
	set_throttle_out(pilot_throttle_scaled,1);//rc_throttle->servo_out

}


			
// update_simple_mode - rotates pilot input if we are in simple mode
//一般模式下，control_in都认为yaw=0
//无头模式下，simple_cos_yaw起飞时记录，转换到yaw=0的位置，再转到当前位置
void update_simple_mode(void)
{
    float rollx, pitchx;
    // exit immediately if no new radio frame or not in simple mode
    if (ap.flags.simple_mode == 0 || !ap.flags.new_radio_frame) //没有新的遥控数据，或者 飞行模式为NONE
	{	
        return;											
    }

    // mark radio frame as consumed
    ap.flags.new_radio_frame = 0;		//标记遥控数据已被使用,接受到遥控数据时置位
    if (ap.flags.simple_mode == 1)		//无头模式
	{//
        // rotate roll, pitch input by -initial simple heading (i.e. north facing)		//cos_yaw,sin_yaw解锁时初始化
        rollx =  g.rc_1.control_in*simple_cos_yaw - g.rc_2.control_in*simple_sin_yaw;	//控制量从起飞的方向转到yaw=0的位置
        pitchx = g.rc_1.control_in*simple_sin_yaw + g.rc_2.control_in*simple_cos_yaw;
    }
	else//超级简单模式，要有gps?
	{			
		// rotate roll, pitch input by -super simple heading (reverse of heading to home)
		rollx = g.rc_1.control_in*super_simple_cos_yaw - g.rc_2.control_in*super_simple_sin_yaw;	//飞无头模式??
		pitchx = g.rc_1.control_in*super_simple_sin_yaw + g.rc_2.control_in*super_simple_cos_yaw;	//
	}

    // rotate roll, pitch input from north facing to vehicle's perspective
    g.rc_1.control_in = rollx*ahrs.cos_yaw + pitchx*ahrs.sin_yaw;	//转到当前位置
    g.rc_2.control_in = -rollx*ahrs.sin_yaw + pitchx*ahrs.cos_yaw;	//
}

void get_pilot_desired_lean_angles(int16_t roll_in, int16_t pitch_in, int16_t *roll_out, int16_t *pitch_out)
{
	static float scaler = 1.0;
	static int16_t angle_max = 0;

	// range check the input
	roll_in = constrain_int16(roll_in, -ROLL_PITCH_INPUT_MAX, ROLL_PITCH_INPUT_MAX);			//最大45度
	pitch_in = constrain_int16(pitch_in, -ROLL_PITCH_INPUT_MAX, ROLL_PITCH_INPUT_MAX);

	// return filtered roll if no scaling required
	if (g.angle_max == ROLL_PITCH_INPUT_MAX) 
	{
		//最大期望角度与默认一样,否则需要归一化处理
		*roll_out = roll_in;
		*pitch_out = pitch_in;
		return;
	}

	// check if angle_max has been updated and redo scaler
	if (g.angle_max != angle_max) 
	{		//更新比例系数
		angle_max = g.angle_max;
		scaler = (float)g.angle_max/(float)ROLL_PITCH_INPUT_MAX; //输入最大时的期望角度
	}

	// convert pilot input to lean angle
	*roll_out = (int16_t)((float)roll_in * scaler);			//
	*pitch_out = (int16_t)((float)pitch_in * scaler);		//
}


float get_pilot_desired_yaw_rate(int16_t stick_angle)
{
    // convert pilot input to the desired yaw rate
    return stick_angle * g.acro_yaw_p;			//比例系数,默认4.5
}


	
#define THROTTLE_IN_MIDDLE 500          // the throttle mid point
int16_t get_pilot_desired_throttle(int16_t throttle_control)
{
    int16_t throttle_out;
    // exit immediately in the simple cases
    if( throttle_control == 0 || g.throttle_mid == 500)	//init_arm_motor中设置了throttle_mid??
		{	//油门量为0或中值为500,(定高模式非500？？)
        return throttle_control;
    }

    // ensure reasonable throttle values
    throttle_control = constrain_int16(throttle_control,0,1000);		//限幅
    g.throttle_mid = constrain_int16(g.throttle_mid,300,700);			//中值限幅??

	
  // check throttle is above, below or in the deadband死区控制
	//悬停油门大于500时,控制量会按比例提高
  if (throttle_control < THROTTLE_IN_MIDDLE)
	{	//小于50%油门
        // below the deadband
   throttle_out = g.throttle_min + ((float)(throttle_control-g.throttle_min))*((float)(g.throttle_mid - g.throttle_min))/((float)(500-g.throttle_min));
  }else if(throttle_control > THROTTLE_IN_MIDDLE) 
	{		//大于50%油门
        // above the deadband
		throttle_out = g.throttle_mid + ((float)(throttle_control-500))*(float)(1000-g.throttle_mid)/500.0f;
  }else
	{
        // must be in the deadband
		throttle_out = g.throttle_mid;
  }
  return throttle_out;
}



// get_smoothing_gain - returns smoothing gain to be passed into attitude_control.angle_ef_roll_pitch_rate_ef_yaw_smooth
//      result is a number from 2 to 12 with 2 being very sluggish and 12 being very crisp
float get_smoothing_gain()//2-12
{
    return (2.0f + (float)g.rc_feel_rp/10.0f);		//.rc_feel_rp默认100
}
