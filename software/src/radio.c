#include "radio.h"
#include "sys.h"
#include "define.h"

#include "AP_Motor.h"
#include "paramter.h"
#include "sbus.h"




// ---------------------------------------------
static void set_failsafe_radio(uint8_t b)		//失控保护
{
	// only act on changes
	// -------------------
	if (failsafe.radio != b)
	{//失控设置改变了

		// store the value so we don't trip the gate twice
		// -----------------------------------------------
		failsafe.radio = b;

		if (failsafe.radio == 0) 
		{//关闭失控保护
			// We've regained radio contact
			// ----------------------------
			//failsafe_radio_off_event();
		}
		else
		{//启动失控保护失控保护
			// We've lost radio contact
			// ------------------------
			//failsafe_radio_on_event();
		}
	}
}


// radio failsafe kicks in after 3 consecutive throttle values below failsafe_throttle_value
#define FS_COUNTER 3        
static void set_throttle_and_failsafe(uint16_t throttle_pwm)
{
	// if failsafe not enabled pass through throttle and exit
	if (g.failsafe_throttle == FS_THR_DISABLED)		////禁止油门保护
	{
		rc_set_pwm(&g.rc_3, throttle_pwm);			//直接设置
		return;
	}

	//check for low throttle value
	if (throttle_pwm < (uint16_t)g.failsafe_throttle_value)			//失控保护油门
	{
		// if we are already in failsafe or motors not armed pass through throttle and exit
		if (failsafe.radio || !apmotor.flags.armed )			//上锁状态或已经在失控保护模式下则忽略
		{//有radio信号,上锁状态
			rc_set_pwm(&g.rc_3, throttle_pwm);			//直接设置
			return;
		}

		// check for 3 low throttle values
		// Note: we do not pass through the low throttle until 3 low throttle values are recieved
		failsafe.radio_counter++;
		if (failsafe.radio_counter >= FS_COUNTER) 
		{		//检测到3个低油门值
			failsafe.radio_counter = FS_COUNTER;  // check to ensure we don't overflow the counter
			set_failsafe_radio(1);
			rc_set_pwm(&g.rc_3, throttle_pwm);			//直接设置
		}
	}
	else
	{		//油门正常,减小失控保护记录
		// we have a good throttle so reduce failsafe counter
		failsafe.radio_counter--;
		if (failsafe.radio_counter <= 0) 
		{
			failsafe.radio_counter = 0;   // check to ensure we don't underflow the counter
			// disengage failsafe after three (nearly) consecutive valid throttle values
			if (failsafe.radio) 
			{
				set_failsafe_radio(0);
			}
		}

		// pass through throttle
		rc_set_pwm(&g.rc_3, throttle_pwm);
	}
}

#define THROTTLE_ZERO_DEBOUNCE_TIME_MS 400
// set_throttle_zero_flag - set throttle_zero flag from debounced throttle control_in
static void set_throttle_zero_flag(int16_t throttle_control)
{
	static uint32_t last_nonzero_throttle_ms = 0;
	uint32_t tnow_ms = millis();

	// if non-zero throttle immediately set as non-zero
	if (throttle_control > 0)
	{
		last_nonzero_throttle_ms = tnow_ms;
		ap.flags.throttle_zero = 0;
	}
	else if (tnow_ms - last_nonzero_throttle_ms > THROTTLE_ZERO_DEBOUNCE_TIME_MS) 
	{
		ap.flags.throttle_zero = 1;
	}
}

void default_dead_zones(void)
{
	rc_set_dead_zone(apmotor.rc_roll,30);
	rc_set_dead_zone(apmotor.rc_pitch,30);
	rc_set_dead_zone(apmotor.rc_pitch,30);
	rc_set_dead_zone(apmotor.rc_yaw,40);
}


//设置从接收机接收的值(radio_in)到control_in的转换类型和大小
void init_rc_in()
{
	rc_set_angle(apmotor.rc_roll, ROLL_PITCH_INPUT_MAX);		//rc.high
	rc_set_angle(apmotor.rc_pitch, ROLL_PITCH_INPUT_MAX);
	rc_set_range(apmotor.rc_throttle, g.throttle_min, g.throttle_max);//rc.low,low_out,high,high_out输出最小最大油门
	rc_set_angle(apmotor.rc_yaw, YAW_INPUT_MAX);											//45度/s

	rc_set_type(apmotor.rc_roll, RC_CHANNEL_TYPE_ANGLE_RAW);
	rc_set_type(apmotor.rc_pitch, RC_CHANNEL_TYPE_ANGLE_RAW);
	rc_set_type(apmotor.rc_yaw, RC_CHANNEL_TYPE_ANGLE_RAW);
	//default_dead_zones();		//paramater中设置
	ap.flags.throttle_zero = 1;
}


void init_rc_out()
{
	apmotor.min_throttle = g.throttle_min;
	
}


////////////////////////////////////////
//有新的遥控信号返回1
uint8_t rcin_rew_input()
{
	static uint32_t sbus_last_update = 0;
	if(sbus_last_update != sbus.last_update)
	{
		sbus_last_update = sbus.last_update;
		return 1;
	}
	return 0;
}

//读取遥控信号
void rcin_read(uint16_t *periods,uint8_t len)
{
	periods[0] = sbus.channel.ch1+600;
	periods[1] = sbus.channel.ch2+600;
	periods[2] = sbus.channel.ch3+600;
	periods[3] = sbus.channel.ch4+600;
	periods[4] = 1500;
	periods[5] = 1500;
	periods[6] = 1500;
	periods[7] = 1500;
}

///////////////////////////////////////////////
void read_radio(void)
{
	static uint32_t last_update_ms = 0;
	uint32_t tnow_ms = millis();
	if(rcin_rew_input())			//有新的输入
	{
		uint16_t periods[8];	//8个输入通道		
		last_update_ms = tnow_ms;
		ap.flags.new_radio_frame = 1;			//在update_simple_mode中被使用

		rcin_read(periods,8);
		rc_set_pwm(&g.rc_1, periods[0]);		//roll,PWM转为期望角度radio_in -> control_in
		rc_set_pwm(&g.rc_2, periods[1]);		//pitch,PWM转为期望角度radio_in -> control_in
		rc_set_pwm(&g.rc_4, periods[3]);		//yaw,PWM转为期望角度radio_in -> control_in

		set_throttle_and_failsafe(periods[2]);			//判断是否失控
		set_throttle_zero_flag(g.rc_3.control_in);		//0输出油门超过一定时间

	}
	else
	{
		uint32_t elapsed = tnow_ms - last_update_ms;		//
		//解锁状态下,不在失控保护下, 启动油门保护时才会调用失控保护
		if ((elapsed >= FS_RADIO_RC_OVERRIDE_TIMEOUT_MS)&&(g.failsafe_throttle && apmotor.flags.armed && !failsafe.radio)) 
		{			
			set_failsafe_radio(1);			//失控保护
		}
	}
}



