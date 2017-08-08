#include "stabilize.h"
#include "AP_Motor.h"
#include "paramter.h"
#include "AttitudeControl.h"


#include "Attitude.h"

void stabilize_run()
{
	int16_t target_roll, target_pitch;
	float target_yaw_rate;
	int16_t pilot_throttle_scaled;

	// if not armed or throttle at zero, set throttle to zero and exit immediately
	if(!apmotor.flags.armed || g.rc_3.control_in <= 0)	//上锁或解锁时没油门
	{
		relax_bf_rate_controller();						//将期望速率设置为当前速率->误差为0
		set_yaw_target_to_current_heading();	//当前期望方向设为当前方向,_angle_ef_target.z
		set_throttle_out(0, 0);							//rc_throttle->servo_out = 0;??
		return;
	}
	
	update_simple_mode();						//修改roll,pitch的control_in,自稳模式下不会进入 simple_mode = 0;
	//输入量归一化处理,得到期望角度
	get_pilot_desired_lean_angles(g.rc_1.control_in, g.rc_2.control_in, &target_roll, &target_pitch);	
	//期望方向速度为杆量直接乘一个系数(默认4.5)
	target_yaw_rate = get_pilot_desired_yaw_rate(g.rc_4.control_in);//最大值为45度/s,*4.5=202	
	//期望油门,死区处理?
//	printf("con_in=%d\n",g.rc_3.control_in);
	pilot_throttle_scaled = get_pilot_desired_throttle(g.rc_3.control_in);	//？？？？乱。。。	
		
	//平滑增益? 2-12,越大反应越快 目标值->rate_bf_target
	angle_ef_roll_pitch_rate_ef_yaw_smooth(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());//gain默认为12
	//
	set_throttle_out(pilot_throttle_scaled,1);//rc_throttle->servo_out

}









	



