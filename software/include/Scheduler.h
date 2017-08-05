#ifndef _SCHEDULER_H_
#define _SCHEDULER_H_

#include "sys.h"

#define SCHEDULER_MAX_TIMER_PROCS	4	
#define SCHEDULER_MAX_TICK_TASK		50
typedef void (*timer_process_t)(void);
typedef void (*task_fn_t)(void);

typedef struct 
{
	task_fn_t function;
	uint16_t interval_ticks;				//tick
	uint16_t max_time_micros;				//最大占用时间
	
}scheduler_tasks_t;

typedef struct
{

	//由定时器调用
	timer_process_t timer_proc[SCHEDULER_MAX_TIMER_PROCS];
	uint16_t num_timer_procs;
	uint8_t timer_suspended:1;
	uint8_t in_timer_proc:1;

	scheduler_tasks_t *tasks;		//任务列表
	uint8_t num_tasks;			//任务数量
	uint16_t tick_counter;		//计数器
	uint16_t last_run[SCHEDULER_MAX_TICK_TASK];//上一次调用tick数
	int8_t current_task;
	
	
	uint32_t task_time_allowed;
	uint32_t task_time_started;
	

	uint32_t spare_micros;		//剩余的时间
	uint8_t spare_ticks;		//剩余的ticks

	uint8_t debug;
}scheduler_t;




extern scheduler_t scheduler;

void scheduler_init(scheduler_tasks_t *tasks, uint8_t num_tasks);
//timer
void scheduler_resume_timer_procs(void);
void scheduler_suspend_timer_procs(void); 
void scheduler_register_timer_process(timer_process_t proc);
void scheduler_timer_event(void);
//tick
void scheduler_tick(void);
void scheduler_run(uint16_t time_available);

#endif
