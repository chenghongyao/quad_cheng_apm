#include "AttitudeControl.h"
#include "AP_Motor.h"
#include "AP_InertialSensor.h"
#include "paramter.h"
#include "Scheduler.h"
#include "AP_State.h"
#include "Compass.h"
#include "stabilize.h"
#include "radio.h"

#include "notify.h"
#include "ano.h"
#include "sbus.h"



attitude_control_t atti_ctrl;
ahrs_dcm_t ahrs;
apmotor_t apmotor;
inertial_sensor_t ins;
paramter_t g;
copter_t ap;	
failsafe_t failsafe;
Compass_t compass;




// Stores initial bearing when armed - initial simple bearing is modified in super simple mode so not suitable
int32_t initial_armed_bearing;		//解锁时的方向,在接解锁时读取 init_arm_motors中初始化,(解锁动作调用)
float simple_cos_yaw = 1.0;				// init_arm_motors中初始化为当前yaw(解锁动作调用)
float simple_sin_yaw;							//无头模式下记录起飞角度

int32_t super_simple_last_bearing;	// init_arm_motors中初始化为当前yaw+180度(解锁动作调用)
float super_simple_cos_yaw = 1.0;
float super_simple_sin_yaw;			//super_simple模式，不用



//
static float G_Dt;
// System Timers
// --------------
static uint32_t fast_loopTimer;		//最近一次进入loop的时间
static uint16_t mainLoop_count;		//loop运行计数

////////////////////////////////////////////////////////////////////////////////
// Altitude
////////////////////////////////////////////////////////////////////////////////
//// The cm/s we are moving up or down based on filtered data - Positive = UP
//static int16_t climb_rate = 0;				//爬升速率,velocity.z
//// The altitude as reported by Sonar in cm - Values are 20 to 700 generally.
//static int16_t sonar_alt;
//static uint8_t sonar_alt_health;    // true if we can trust the altitude from the sonar
//static float target_sonar_alt;      // desired altitude in cm above the ground

//static int32_t baro_alt;            // barometer altitude in cm above home
//static float baro_climbrate;        // barometer climbrate in cm/s


///////////////////////////////////////////////////////////////////////////////////////
static void fast_loop(void);
static void read_AHRS(void);
static void set_servos_4(void);
void rc_loop(void);
void throttle_loop(void);
void arm_motors_check(void);
void update_notify(void);
void one_hz_loop(void);
///////////////////////////////////////////////////////////////////////////////////////

/*
scheduler table - all regular tasks apart from the fast_loop()
should be listed here, along with how often they should be called
(in 10ms units) and the maximum time they are expected to take (in
microseconds)
1    = 100hz
2    = 50hz
4    = 25hz
10   = 10hz
20   = 5hz
33   = 3hz
50   = 2hz
100  = 1hz
1000 = 0.1hz
*/
static scheduler_tasks_t scheduler_tasks[] =
{
	{rc_loop,					1,				100},		//100Hz,读取遥控信号
	{arm_motors_check, 	10,     10 },		//10Hz,上锁,解锁动作检测,10Hz
	{update_notify,			2,			100},		//50Hz,更新灯
	{one_hz_loop,				100,		420},		//pre_arm_checks
};





void setup()
{
	param_setup_and_load();		//加载系统参数
	
	if(g.esc_calibrate)
	{
		while(1)
		{
			uint8_t i;
			uint16_t periods[8];	//8个输入通道		
			if(rcin_rew_input())
			{
				rcin_read(periods,8);
				for (i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++)
				{
					motor_write(i,periods[2]);	//油门通道
				}
			}
			
		}
		
	}
	
	scheduler_init(scheduler_tasks,sizeof(scheduler_tasks)/sizeof(scheduler_tasks[0]));				//定时器??
	notify.flags.initialising = 1;
	inertial_sensor_init();
	ahrs_dcm_init();
	//Compass_init();
	apmotor_init(&g.rc_1,&g.rc_2,&g.rc_3,&g.rc_4);
	attitude_control_init();
	
	init_rc_in();
	init_rc_out();
	
	
	read_radio();
	

	
	notify.flags .initialising = 0;
	
}



extern uint16_t motor_monitor[4];
void loop()
{
	uint32_t timer;
	uint32_t time_available;
	
	inertial_sensor_wait_for_sample();			
	timer = micros();	
	// used by PI Loops
	G_Dt                    = (float)(timer - fast_loopTimer) / 1000000.f;		//调用周期
	fast_loopTimer          = timer;

	// for mainloop failure monitoring
	mainLoop_count++;		

	// Execute the fast loop
	// ---------------------
	fast_loop();
	//ANO_DT_Send_MotoPWM(motor_monitor[0],motor_monitor[1],motor_monitor[2],motor_monitor[3],0,0,0,0);
	//ANO_DT_Send_Sensor(apmotor.rc_roll->servo_out,apmotor.rc_pitch->servo_out,apmotor.rc_yaw->servo_out,apmotor.rc_throttle->servo_out,0,0,0,0,0);

	//ANO_DT_Send_Sensor(atti_ctrl.rate_bf_target.x,ahrs.omega.x*AC_ATTITUDE_CONTROL_DEGX100,apmotor.rc_roll->servo_out,apmotor.rc_throttle->servo_out,0,0,0,0,0);
	//ANO_DT_Send_Sensor((float)g.rc_1.control_in,(float)g.rc_2.control_in,(float)g.rc_4.control_in,g.rc_3.control_in,0,0,0,0,0);
	//ANO_DT_Send_Sensor(ins.accel.x*100,ins.accel.y*100,ins.accel.z*100,ins.gyro.x*100,ins.gyro.y*100,ins.gyro.z*100,0,0,0);
	//ANO_DT_Send_Sensor(ahrs.omega_P.x*1000,ahrs.omega_P.y*1000,ahrs.omega_P.z*1000,0,0,0 ,0,0,0);
	//ANO_DT_Send_Sensor(ahrs.omega_P.x*1000,ahrs.omega_P.y*1000,ahrs.omega_P.z*1000,ahrs.err_len*1000,0,0 ,0,0,0);
	//ANO_DT_Send_Status((float)ahrs.roll_sensor/100,(float)ahrs.pitch_sensor/100,ahrs.yaw_sensor/100,0,0,0);
	// tell the scheduler one tick has passed
	//ANO_DT_Send_Status((float)g.rc_1.control_in/100,(float)g.rc_2.control_in/100,(float)g.rc_4.control_in/100,g.rc_3.control_in,0,0);
	scheduler_tick();	
	time_available = (timer + MAIN_LOOP_MICROS) - micros();//运行10ms
	scheduler_run(time_available);
}




void fast_loop()
{
	read_AHRS();		//--
	rate_controller_run();		//!速率控制
	set_servos_4();						//
	stabilize_run();		//姿态 control_in -> rate_bf_target,油门 control_in -> servo_out
}


static void read_AHRS()
{
	ahrs_dcm_update();
}

static void set_servos_4()
{
	apmotor_output();
}


//=================================================================
void rc_loop()//100Hz
{
	read_radio();
}

//10Hz，解锁，上锁动作检测？？？？？？？
void arm_motors_check(void)
{
	static int16_t arming_counter = 0;
	int16_t tmp;
	
	if (g.rc_3.control_in > 0)		//油门非0，直接退出
	{
		arming_counter = 0;
		return;
	}

	tmp = g.rc_4.control_in;	//方向通道值
	if (tmp > 4000)				//contol_in对radio_in做了映射，最大是4500
	{
		if (arming_counter <= ARM_DELAY)//10s启动自动微调
		{
			arming_counter++;
		}

		if (arming_counter == ARM_DELAY && !apmotor.flags.armed)	//检测到解锁动作2s
		{
			pre_arm_checks(1);		//检查遥控器是否校准，检查所有传感器，
			if (ap.flags.pre_arm_check && arm_checks(1))//解锁检测成功
			{
				if (!init_arm_motors())		
				{
					arming_counter = 0;	//解锁成功
				}
			}
			else//解锁失败
			{
				arming_counter = 0;
			}
		}
	}
	else if (tmp < -4000)//上锁
	{
		if (arming_counter <= DISARM_DELAY)
		{
			arming_counter++;
		}

		if (arming_counter == DISARM_DELAY && apmotor.flags.armed)
		{
			init_disarm_motors();//上锁
		}
	}
	else//非解锁或上锁
	{
		arming_counter = 0;
	}
}



void init_simple_bearing()
{
		initial_armed_bearing = ahrs.yaw_sensor;
    // capture current cos_yaw and sin_yaw values
    simple_cos_yaw = ahrs.cos_yaw; //当前的方向
    simple_sin_yaw = ahrs.sin_yaw;

    // initialise super simple heading (i.e. heading towards home) to be 180 deg from simple mode heading
    super_simple_last_bearing = wrap_360_cd(ahrs.yaw_sensor+18000);		//当前方向加180度??
    super_simple_cos_yaw = simple_cos_yaw;		
    super_simple_sin_yaw = simple_sin_yaw;

}

// update_auto_armed - update status of auto_armed flag
static void update_auto_armed()
{
	// disarm checks
	if (ap.flags.auto_armed)
	{		//允许自动解锁
		// if motors are disarmed, auto_armed should also be false
		if (!apmotor.flags.armed) 				//一旦上锁，就关闭自动解锁
			{			//上锁了
				set_auto_armed(0);		//关闭自动解锁
				return;
			}
			//还未上锁，如果，如果油门时0，也关闭自动解锁
		// if in stabilize or acro flight mode and throttle is zero, auto-armed should become false
		//没有上锁,子自稳和特技模式下，如果油门时0，就关闭自动解锁
		if (ap.flags.throttle_zero && !failsafe.radio) 
		{//自稳和特技模式且如果信号没有丢失且???关闭自动解锁
			set_auto_armed(0);
		}

	}
	else
		{	//不允许自动解锁??
		// arm checks
		// if motors are armed and throttle is above zero auto_armed should be true
		if (apmotor.flags.armed && !ap.flags.throttle_zero) //解锁了且油门非0;
		{
			set_auto_armed(1);//解锁了且油门非0,使能自动解锁
		}

	}
}

//50Hz
void throttle_loop()
{
	update_auto_armed();		//?????

}
void update_notify(void)
{
	notify_update();
}
void one_hz_loop(void)
{
 // perform pre-arm checks & display failures every 30 seconds
    static uint8_t pre_arm_display_counter = 15;
	
	
    pre_arm_display_counter++;
    if (pre_arm_display_counter >= 5) 
		{	
			pre_arm_checks(1);				//所有传感器自检
			pre_arm_display_counter = 0;
    }
		else
		{
      pre_arm_checks(1);
    }

}
