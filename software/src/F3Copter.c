#include "AttitudeControl.h"
#include "AP_Motor.h"
#include "AP_InertialSensor.h"
#include "Scheduler.h"
#include "AP_State.h"
#include "Compass.h"
#include "InertialNav.h"
#include "PosControl.h"

#include "paramter.h"
#include "stabilize.h"
#include "althold.h"
#include "radio.h"
#include "notify.h"
#include "ano.h"
#include "sbus.h"
#include "barometer.h"
#include "inertia.h"
#include "defines.h"
#include "land_detector.h"

inertial_sensor_t ins;
InertialNav_t inav;
ahrs_dcm_t ahrs;
attitude_control_t atti_ctrl;
pos_control_t pos_ctrl;

barometer_t barometer;


apmotor_t apmotor;
paramter_t g;
copter_t ap;	
failsafe_t failsafe;
Compass_t compass;

static int8_t control_mode = ALT_HOLD;		//全局变量,控制模式,默认为自稳
uint8_t oldSwitchPosition;
uint8_t *flight_modes = g.flight_modes;
// Stores initial bearing when armed - initial simple bearing is modified in super simple mode so not suitable
int32_t initial_armed_bearing;		//解锁时的方向,在接解锁时读取 init_arm_motors中初始化,(解锁动作调用)
float simple_cos_yaw = 1.0;				// init_arm_motors中初始化为当前yaw(解锁动作调用)
float simple_sin_yaw;							//无头模式下记录起飞角度

int32_t super_simple_last_bearing;	// init_arm_motors中初始化为当前yaw+180度(解锁动作调用)
float super_simple_cos_yaw = 1.0;
float super_simple_sin_yaw;			//super_simple模式，不用
int16_t desired_climb_rate;


//
float G_Dt;



//
////////////////////////////////////////////////////////////////////////////////
// Throttle variables
////////////////////////////////////////////////////////////////////////////////
//static float throttle_avg;     // g.throttle_cruise as a float
int16_t desired_climb_rate;    //调试用,期望爬升速率,rc_throttle.control_in对最高期望速率归一化


// System Timers
// --------------
static uint32_t fast_loopTimer;		//最近一次进入loop的时间
static uint16_t mainLoop_count;		//loop运行计数

////////////////////////////////////////////////////////////////////////////////
// Altitude
////////////////////////////////////////////////////////////////////////////////
// The cm/s we are moving up or down based on filtered data - Positive = UP
//static int16_t climb_rate = 0;				//爬升速率,velocity.z
// The altitude as reported by Sonar in cm - Values are 20 to 700 generally.
int16_t sonar_alt;
uint8_t sonar_alt_health;    // true if we can trust the altitude from the sonar
float target_sonar_alt;      // desired altitude in cm above the ground

int32_t baro_alt;            // barometer altitude in cm above home
float baro_climbrate;        // barometer climbrate in cm/s


///////////////////////////////////////////////////////////////////////////////////////
static void fast_loop(void);
static void read_AHRS(void);
static void set_servos_4(void);
static void update_flight_mode(void);
void rc_loop(void);
void throttle_loop(void);
void arm_motors_check(void);
void update_notify(void);
void one_hz_loop(void);
void update_altitude(void);
void run_nav_updates(void);
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
	{rc_loop,					1,			100},		//100Hz,读取遥控信号
	{arm_motors_check, 			10,			10 },		//10Hz,上锁,解锁动作检测,10Hz
	{update_notify,				2,			100},		//50Hz,更新灯
	{one_hz_loop,				100,		420},		//pre_arm_checks
	{update_altitude,			10,			1000 },		//更新高度10Hz,100ms??(读取气压计,超声波)
	{throttle_loop, 			2,		  450 },		//油门更新50Hz??:读取高度和爬升速率到climb_rate,current_loc,检查是否降落或自动解锁
};

void setup()
{
	param_setup_and_load();		//加载系统参数
	//电调校准
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
	scheduler_init(scheduler_tasks,sizeof(scheduler_tasks)/sizeof(scheduler_tasks[0]));				
	notify.flags.initialising = 1;
	inertial_sensor_init();
	ahrs_dcm_init();
	apmotor_init(&g.rc_1,&g.rc_2,&g.rc_3,&g.rc_4);
	barometer_init();
	
	attitude_control_init();
	pos_control_init();
	InertialNav_init();


	init_rc_in();
	init_rc_out();
	

	notify.flags .initialising = 0;
	//printf("ready");
}


//extern float ms5611_temperature;				//温度
//extern float ms5611_pressure;				//温度
extern uint16_t motor_monitor[4];
extern float hist_base,hist_push;
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
//	ANO_DT_Send_Sensor(barometer.Temp,barometer.Press,baro_alt ,apmotor.rc_roll->servo_out,apmotor.rc_throttle->servo_out,0,0,0,0);
	//ANO_DT_Send_Sensor((float)g.rc_1.control_in,(float)g.rc_2.control_in,(float)g.rc_4.control_in,g.rc_3.control_in,0,0,0,0,0);
	//ANO_DT_Send_Sensor(ins.accel.x*100,ins.accel.y*100,ins.accel.z*100,ins.gyro.x*100,ins.gyro.y*100,ins.gyro.z*100,0,0,0);
	//ANO_DT_Send_Sensor(ahrs.omega_P.x*1000,ahrs.omega_P.y*1000,ahrs.omega_P.z*1000,0,0,0 ,0,0,0);
	//ANO_DT_Send_Sensor(ahrs.omega_P.x*1000,ahrs.omega_P.y*1000,ahrs.omega_P.z*1000,ahrs.err_len*1000,0,0 ,0,0,0);
	//ANO_DT_Send_Status((float)ahrs.roll_sensor/100,(float)ahrs.pitch_sensor/100,ahrs.yaw_sensor/100,0,0,0);				//2ms?
	// tell the scheduler one tick has passed
	//ANO_DT_Send_Status((float)g.rc_1.control_in/100,(float)g.rc_2.control_in/100,(float)g.rc_4.control_in/100,g.rc_3.control_in,0,0);
	ANO_DT_Send_Sensor(pos_ctrl.pos_target.z,inav.position.z ,pos_ctrl.vel_target.z,inav.velocity.z,apmotor.rc_throttle->servo_out,barometer.altitude ,baro_climbrate,0,0);
	//ANO_DT_Send_Sensor(desired_climb_rate,pos_ctrl.pos_target.z,inav.velocity.z ,baro_climbrate,0,0, 0, 0, 0);
	
	scheduler_tick();	
	time_available = (timer + MAIN_LOOP_MICROS) - micros();//运行10ms
	scheduler_run(time_available);
}





void fast_loop()
{
	read_AHRS();								//--
	rate_controller_run();			//!速率控制
	set_servos_4();							//
	read_inertia();							//更新_velocity,_position 
	update_flight_mode();
}


static void read_AHRS()
{
	ahrs_dcm_update();
}

static void set_servos_4()
{
	apmotor_output();
}



static void update_flight_mode()
{
	switch (control_mode)
	{
	case STABILIZE:
		stabilize_run();				//姿态 control_in -> rate_bf_target,油门 control_in -> servo_out
		break;
	case ALT_HOLD:
		althold_run();
		break;
	default:
		break;
	}
}

//=================================================================

uint8_t readSwitch(void)
{
	int16_t pulsewidth;
//	degub("control_in=%d",g.rc_5.control_in);
	pulsewidth = g.rc_5.control_in;   // default for Arducopters
	if (pulsewidth < 333) return 0;
	if (pulsewidth < 666) return 1;
	return 2;                               // Hardware Manual
}
uint8_t set_mode(uint8_t mode)
{
   // boolean to record if flight mode could be set
   uint8_t success = 0;
   uint8_t ignore_checks = !apmotor.flags.armed;   // allow switching to any mode if disarmed.  We rely on the arming check to perform
																									//上锁状态下可以忽略检测
   // return immediately if we are already in the desired mode
   if (mode == control_mode) 
   {		
       return 1;				//模式没有改变
   }
	 
   switch(mode) 
   {
       case STABILIZE:
         success = stabilize_init(ignore_checks);	
			   debug("stabilize");
           break;
       case ALT_HOLD:
			   success = althold_init(ignore_checks);
			   debug("althold");
				break;
	   default:
           success = 0;
           break;
   }
   // update flight mode
	if (success) 
	{
       // perform any cleanup required by previous flight mode
       control_mode = mode;
   }
   // return success or failure
   return success;
}


#define CONTROL_SWITCH_COUNTER  20  // 20 iterations at 100hz (i.e. 2/10th of a second) at a new switch position will cause flight mode change
void read_control_switch()
{
   static uint8_t switch_counter = 0;
	static uint8_t first = 1;
   uint8_t switchPosition = readSwitch();		//模式选择开关位置,第5通道
   // has switch moved?
   // ignore flight mode changes if in failsafe ,检测到改变200ms
	if ((first || oldSwitchPosition != switchPosition) && !failsafe.radio && failsafe.radio_counter == 0) //模式改变
	{
		switch_counter++;
		if(first || switch_counter >= CONTROL_SWITCH_COUNTER) 
		{
			first = 0;
			oldSwitchPosition       = switchPosition;
			switch_counter          = 0;
			// set flight mode and simple mode setting
			//根据拨杆位置获取飞行模式(在参数表中)
			if (set_mode(flight_modes[switchPosition]))//设置飞行模式成功->control_mode
			{		
					
			}
		}
	}
	else
	{
       // reset switch_counter if there's been no change
       // we don't want 10 intermittant blips causing a flight mode change
       switch_counter = 0;
	}
}

void rc_loop()//100Hz
{
	read_radio();
	read_control_switch();
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



void update_notify(void)
{
	notify_update();
}

void update_baro_altitude()
{
	barometer_read();//处理气压,温度->Temp,Press
	baro_alt = barometer_get_altitude();
	baro_climbrate = barometer_get_climbrate();
}



//更新气压高度,速率,超声波高度
void update_altitude()
{
	update_baro_altitude();
}




void one_hz_loop(void)
{
	// perform pre-arm checks & display failures every 5 seconds
	static uint8_t pre_arm_display_counter = 15;
	pre_arm_display_counter++;
	if (pre_arm_display_counter >= 5)
	{
		pre_arm_checks(1);				//
		pre_arm_display_counter = 0;
	}
	else
	{
		pre_arm_checks(1);
	}
}

/////////////////////////////////////////////////////////////////////////////////////

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


//50Hz
void throttle_loop()
{
//	climb_rate = inav.velocity.z;
	update_land_detector();
//	update_auto_armed();		//?????

}

