#include "AP_InertialSensor.h"
#include "sys.h"
#include "Scheduler.h"
	


// MPU6000 accelerometer scaling
#define MPU6000_ACCEL_SCALE_1G    (GRAVITY_MSS / 4096.0f)			//量程8G


void inertial_sensor_wait_for_sample()			//等待至少2.5ms
{
	uint32_t now;
	
	if(ins.have_sample)		//样本还未使用
	{
		return ;
	}

	now = micros();
	if(ins.next_sample_usec == 0 && ins.delta_time <=0)
	{
		ins.last_sample_usec = now - ins.sample_period_usec;
		ins.next_sample_usec = now + ins.sample_period_usec;
		goto check_sample;
	}

	//等待样本读取完成
	if((ins.next_sample_usec - now) < ins.sample_period_usec)
	{		
		uint32_t wait_usec = ins.next_sample_usec - now;	
		
		if(wait_usec > 200)
		{
			
			delay_us(wait_usec);
		}
		ins.next_sample_usec += ins.sample_period_usec;
	}
	else		//读取周期过长??
	{
		ins.next_sample_usec = now + ins.sample_period_usec;
	}
	
check_sample:	
	now = micros();
	ins.delta_time = (float)(now - ins.last_sample_usec)*1.0e-6f;
	ins.last_sample_usec = now;
	ins.have_sample = 1;
}




static void _rotate_and_offset_accel(vector3f_t *accel)
{
	vector3f_copy(accel,&ins.accel);
	vector3f_rotate(&ins.accel,ins.board_orientation);
	ins.accel.x *= ins.accel_scale.x;
	ins.accel.y *= ins.accel_scale.y;
	ins.accel.z *= ins.accel_scale.z;
	vector3f_minus(&ins.accel,&ins.accel_offset,NULL);		
}

static void _rotate_and_offset_gyro(vector3f_t *gyro)
{
	vector3f_copy(gyro,&ins.gyro);
	vector3f_rotate(&ins.gyro,ins.board_orientation);
	vector3f_minus(&ins.gyro,&ins.gyro_offset,NULL);		
}


uint8_t inertial_sensor_calibrate_accel()
{
	static vector3f_t accel_sum = {0.0,0.0,0.0};
	static uint16_t cnt = 0;
	if(cnt<100)
	{
		accel_sum.x += ins.accel.x;
		accel_sum.y += ins.accel.y;
		accel_sum.z += (ins.accel.z + GRAVITY_MSS);
		cnt ++;
		return 0;
	}
	else
	{
		vector3f_scale(&accel_sum,1.0f/cnt,&ins.accel_offset);
		vector3f_zero(&accel_sum);
		cnt = 0;
		ins.calibrated = 1;
		return 1;		//校准完成
	}
}



uint8_t inertial_sensor_calibrate2_accel()
{
	static vector3f_t accel_sum = {0.0,0.0,0.0};
	static uint16_t cnt = 0;
	if(cnt<100)
	{
		accel_sum.x += ins.accel.x;
		accel_sum.y += ins.accel.y;
		accel_sum.z += (ins.accel.z + GRAVITY_MSS);
		cnt ++;
		return 0;
	}
	else
	{
		vector3f_scale(&accel_sum,1.0f/cnt,&ins.accel_offset);
		vector3f_zero(&accel_sum);
		cnt = 0;
		ins.calibrated = 1;
		return 1;		//校准完成
	}
}

uint8_t gyro_calibrated_ok_all()
{
	return 1;
}


//读取，求平均，转物理单位，旋转？
uint8_t inertial_sensor_update()
{
	vector3f_t accel,gyro;
	uint16_t num_samples;
	
	if(ins.sum_count < ins.sample_count)		//样本不够
	{
		return 1;
	}
	
	//防止再采样,
	scheduler_suspend_timer_procs();	
	//读取数据
	vector3f_set(&gyro,ins.gyro_sum.x,ins.gyro_sum.y,ins.gyro_sum.z);
	vector3f_set(&accel,ins.accel_sum.x,ins.accel_sum.y,ins.accel_sum.z);
	num_samples = ins.sum_count;
	//复位
	vector3f_zero(&ins.gyro_sum);		
	vector3f_zero(&ins.accel_sum);
	ins.sum_count = 0;
	//继续采样
	scheduler_resume_timer_procs();			
	
	//转换，矫正
	vector3f_scale(&gyro,ins.gyro_scale/num_samples,NULL);	//求平均，转物理单位,rad
	_rotate_and_offset_gyro(&gyro);
	
	vector3f_scale(&accel,MPU6000_ACCEL_SCALE_1G/num_samples,NULL);//求平均，转物理单位,m/s2
	_rotate_and_offset_accel(&accel);
	

	if(vector3f_iszero(&ins.accel_offset))
	{
		inertial_sensor_calibrate_accel();
		return 0;
	}
	
	ins.have_sample = 0;		//100Hz,不设置也行??,
	return 1;
}

static void _poll_data()
{
	int16_t acc_adc[3];
	int16_t gyro_adc[3];

	if(!mpu6050_getAdcAll(acc_adc,gyro_adc))return;
	
	ins.accel_sum.x += (float)acc_adc[1];
	ins.accel_sum.y += (float)acc_adc[0];
	ins.accel_sum.z -= (float)acc_adc[2];
	
	ins.gyro_sum.x += (float)gyro_adc[1];
	ins.gyro_sum.y += (float)gyro_adc[0];
	ins.gyro_sum.z -= (float)gyro_adc[2];
	ins.sum_count ++;
	if(ins.sum_count == 0)
	{
		vector3f_zero(&ins.accel_sum);
		vector3f_zero(&ins.gyro_sum);
	}
}



void inertial_sensor_init_sensor()
{

	 mpu6050_init();
	scheduler_suspend_timer_procs();
	scheduler_register_timer_process(_poll_data);
	scheduler_resume_timer_procs();
}


void inertial_sensor_init()
{
	ins.sample_count = 2;			//一次处理所需样本数
	ins.sample_max = 0;
	ins.sum_count = 0;
	vector3f_zero(&ins.accel_sum);
	vector3f_zero(&ins.gyro_sum);
	
	
	ins.delta_time  = 0;
	ins.next_sample_usec = 0;
	ins.last_sample_usec = 0;
	ins.sample_period_usec = 10000;	//周期10ms
	ins.have_sample = 0;
	

	ins.board_orientation = ROTATION_NONE;
	ins.gyro_scale = (0.0174532f / 16.4f);			//adc->rad 量程+/-2000deg
	ins.calibrated = 0;													//未校准
	ins.gyro_cal_ok = 1;
	//!!!!!!!!!!!
	vector3f_set(&ins.accel_scale,1.0,1.0,1.0);
	vector3f_set(&ins.accel_offset,0,0,0);
	vector3f_set(&ins.gyro_offset,0.01,-0.03,0.0);
	
	inertial_sensor_init_sensor();
}























