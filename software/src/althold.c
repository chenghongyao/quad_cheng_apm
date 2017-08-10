#include "althold.h"
#include "AP_Motor.h"
#include "paramter.h"
#include "AttitudeControl.h"
#include "PosControl.h"
#include "InertialNav.h"
#include "AP_State.h"
#include "Attitude.h"
#include "PosControl.h"
extern float G_Dt;



uint8_t althold_init(uint8_t ignore_check)
{
	// initialize vertical speeds and leash lengths
	pos_control_set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
	pos_control_set_accel_z(g.pilot_accel_z);
	// initialise altitude target to stopping point
	pos_control_set_target_to_stopping_point_z();
	return 1;
}

void althold_run()
{
	int16_t target_roll, target_pitch;
	float target_yaw_rate;
	int16_t target_climb_rate;
	// if not armed or throttle at zero, set throttle to zero and exit immediately
	if (!apmotor.flags.armed || g.rc_3.control_in <= 0)	//上锁,0油门
	{
		relax_bf_rate_controller();								//将期望速率设置为当前速率->误差为0
		set_yaw_target_to_current_heading();			//当前期望方向设为当前方向,_angle_ef_target.z
		set_throttle_out(0, 0);							//rc_throttle->servo_out = 0;
		pos_ctrl.pos_target.z = inav.position.z;
		return;
	}

	update_simple_mode();						//修改roll,pitch的control_in,自稳模式下不会进入 simple_mode = 0;
	//输入量归一化处理,得到期望角度
	get_pilot_desired_lean_angles(g.rc_1.control_in, g.rc_2.control_in, &target_roll, &target_pitch);
	//期望方向速度为直接乘一个系数(默认4.5)
	target_yaw_rate = get_pilot_desired_yaw_rate(g.rc_4.control_in);//最大值为45度/s,*4.5=202	
	//rc_throttle.control_in 对最高期望速率归一化
	target_climb_rate = get_pilot_desired_climb_rate(g.rc_3.control_in);
	
	//在地上期望起飞,清除落地标志
	if (ap.flags.land_complete && target_climb_rate > 0)
	{
		set_land_complete(0);
		set_throttle_takeoff();			//起飞准备:slow_start = 1,关闭前馈??,设置初始积分值
	}

	if (ap.flags.land_complete)//在地上,target_climb_rate<=0
	{
		relax_bf_rate_controller();																				//将期望速率设置为当前速率->误差为0
		set_yaw_target_to_current_heading();															//当前期望方向设为当前方向,_angle_ef_target.z
		set_throttle_out(get_throttle_pre_takeoff(g.rc_3.control_in), 0);	//最大值1/2 throttle_mid,输入对其归一化
		pos_ctrl.pos_target.z = inav.position.z;
	}
	
	else//不在地上
	{
		//平滑增益? 2-12,越大反应越快 目标值->rate_bf_target
		angle_ef_roll_pitch_rate_ef_yaw_smooth(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());//gain默认为12
		pos_control_set_alt_target_from_climb_rate(target_climb_rate, G_Dt,0);			//从爬升速率获取期望高度(积分)
		pos_control_update_z_controller();			//z轴控制
	}
}

