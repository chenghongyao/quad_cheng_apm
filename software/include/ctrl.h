#ifndef _CTR_H_
#define _CTR_H_

#include "sys.h"


enum
{
	MO_HL =	0,
	MO_HR = 1,
	MO_TL = 2,
	MO_TR  = 3,
};

typedef struct
{
	float kp;
	float kd;
	float ki;
	float kdamp;
	float out;
	
	float err;
	float err_old;
	float err_d;
	float err_i;
	float err_weight;
	float damp;
	float eliminate_i;
}pid_t;





typedef struct
{
	
	float thr_weight;				//当前油门占总油门的比重
	float thr_takeoff;
	
	
	float angle_rate_target[3];
	float angle_target[3];
	float angle_offset[3];
	float alt_rate_target;					//桨油门转为上身速率
	float alt_target;
	float px_target;
	float vx_target;
	float py_target;
	float vy_target;
	
	float pos_roll;
	float pos_pitch;
	
	
	float angle_rate_elliminate_i;
	float angle_elliminate_i;
	float alt_elliminate_i;
	float alt_rate_elliminate_i;

	float angle_rate_FB;
	
	float gyro_old[3];

	
	float thro;							//0-1000??
	float roll;
	float pitch;
	float yaw;
	
	
	float motor[4];
}controller_t;

extern controller_t mCtrl;



#define CTRL_ATTI_MAX_RATE			300
#define CTRL_YAW_MAX_RATE				150
#define ANGLE_TO_MAX_RATE				30
#define CTRL_ATTI_MAX_ANGLE			30

#define MAX_VERTICAL_SPEED_UP  5000			//最大上升速度,mm/s								
#define MAX_VERTICAL_SPEED_DW  3000			//最大下降速度mm/s

#define CTRL_TAKEOFF_SPEED		 100				//自动起飞速度
#define CTRL_TAKEOFF_HEIGHT		 500				//自动起飞高度
#define CTRL_LAND_HEIGHT		 	 50				//自动起飞高度
#define CTRL_LAND_SPEED		 		200				//自动起飞速度

#define CTRL_ATTIRATE_INT_LIMIT 		0.5f *CTRL_ATTI_MAX_RATE		//内环积分幅度
#define CTRL_ATTI_INT_LIMIT 				0.5f *CTRL_ATTI_MAX_ANGLE		//内环积分幅度

#define CTRL_ALT_INT_LIMIT 					200		//内环积分幅度
#define CTRL_ALTRATE_INT_LIMIT 			0.5*MAX_VERTICAL_SPEED_DW		//内环积分幅度



#define CTRL_MAX_PWM								1000//油门通道最大占比80%，留20%给控制量
#define CTRL_THRO_MAX								0.8f
#define CTRL_THRO_READY							0.2f
#define CTRL_THRO_TAKEOFF_LIMIT			0.7f*CTRL_MAX_PWM		//起飞油门限制



extern pid_t pidRollRate,pidPitchRate,pidYawRate;
extern pid_t pidRoll,pidPitch,pidYaw;
extern pid_t pidAlt,pidAltRate;
extern pid_t pidPosY,pidRateY;
extern pid_t pidPosX,pidRateX;

void ctrl_dealAttiRate(float dt);
void ctrl_dealAtti(float dt);
void ctrl_init(void);

void ctrl_reset_pid(pid_t *pid);
#endif

