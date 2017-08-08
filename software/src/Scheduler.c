#include "Scheduler.h"
#include "sys.h"





scheduler_t scheduler;

void scheduler_init(scheduler_tasks_t *tasks, uint8_t num_tasks)
{
	uint16_t i;
	//timer process
	scheduler.num_timer_procs = 0;
	for (i = 0; i < SCHEDULER_MAX_TIMER_PROCS; i++)
	{
		scheduler.timer_proc[i] = NULL;
	}
	scheduler.timer_suspended = 0;
	scheduler.in_timer_proc = 0;

	//tick proc
	scheduler.tasks = tasks;
	scheduler.num_tasks = num_tasks;
	for (i = 0; i < SCHEDULER_MAX_TICK_TASK; i++)
	{
		scheduler.last_run[i] = 0;
	}
	scheduler.tick_counter = 0;
	scheduler.current_task = -1;

}


///////////////////////////////////////////////////////////////////////////
void scheduler_suspend_timer_procs() 
{
    scheduler.timer_suspended = 1;
}

void scheduler_resume_timer_procs() 
{
    scheduler.timer_suspended = 0;
	
//    if (_timer_event_missed == true) 
//		{
//        _run_timer_procs(false);
//        _timer_event_missed = false;
//    }
}


void scheduler_register_timer_process(timer_process_t proc)
{
	uint16_t i;
	
	for (i = 0; i < scheduler.num_timer_procs;i++)	//重复注册
	{
		if (scheduler.timer_proc[i] == proc)
			return;
	}

	if (scheduler.num_timer_procs < SCHEDULER_MAX_TIMER_PROCS)
	{
		//printf("timer register\n");
		scheduler.timer_proc[scheduler.num_timer_procs] = proc;
		scheduler.num_timer_procs++;
	}
}


//有定时器调用,500Hz
void scheduler_timer_event()
{
	scheduler.in_timer_proc = 1;
	if(!scheduler.timer_suspended)
	{
		uint16_t i;
		for(i=0;i<scheduler.num_timer_procs;i++)
		{
			if(scheduler.timer_proc[i] != NULL)
			{
				scheduler.timer_proc[i]();
				//printf("proc");
			}
				
		}
	}
	scheduler.in_timer_proc = 0;
}


void scheduler_run(uint16_t time_available)
{
	uint32_t now = micros();
	uint8_t i;
	for ( i = 0; i<scheduler.num_tasks; i++)		//运行所有任务
	{
		uint16_t dt = scheduler.tick_counter - scheduler.last_run[i];		//
		uint16_t interval_ticks = scheduler.tasks[i].interval_ticks;		//调用间隔
		if (dt >= interval_ticks) 
		{
			// this task is due to run. Do we have enough time to run it?						
			scheduler.task_time_allowed = scheduler.tasks[i].max_time_micros;
			if (scheduler.task_time_allowed <= time_available)//时间足够		
			{
				task_fn_t func;
				uint32_t time_taken;
				// run it
				scheduler.task_time_started = now;			//记录开始时间

				func = (task_fn_t)scheduler.tasks[i].function;			//运行
				scheduler.current_task = i;
				func();
				scheduler.current_task = -1;

				scheduler.last_run[i] = scheduler.tick_counter;			//记录调用tick
				time_taken = micros() - scheduler.task_time_started;//实际运行时间

				//if (time_taken > scheduler.task_time_allowed) //运行超时
				//{
				//	// the event overran!
				//	if (scheduler.debug > 2) {
				//		printf(PSTR("Scheduler overrun task[%u] (%u/%u)\n"),
				//			(unsigned)i,
				//			(unsigned)time_taken,
				//			(unsigned)scheduler.task_time_allowed);
				//	}
				//}

				if (time_taken >= time_available)	//时间用完了,退出
				{
					goto update_spare_ticks;
				}
				
				time_available -= time_taken;		//未用完,继续
			}
		}
	}
	
	scheduler.spare_micros += time_available;//剩余时间
update_spare_ticks:
	scheduler.spare_ticks++;
	if (scheduler.spare_ticks == 32) 
	{
		scheduler.spare_ticks /= 2;
		scheduler.spare_micros /= 2;
	}
}

void scheduler_tick()
{
	scheduler.tick_counter ++;
}















