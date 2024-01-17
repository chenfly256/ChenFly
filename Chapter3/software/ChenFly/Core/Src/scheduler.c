#include "scheduler.h"
#include "led.h"
#include "voltage.h"
#include "motor.h"
#include "sbus.h"

uint8_t task_num;

static scheduler_task_t scheduler_task[] =
{
	{motor_speed_update, 	1, 		0},
	{sbus_update, 			10, 	0},
	{send_channels, 		100, 	0},
	{led_blink, 			500, 	0},
	{voltage_read, 			200, 	0},
};

void Scheduler_init(void)
{
	task_num = sizeof(scheduler_task)/sizeof(scheduler_task_t);
}

void Scheduler_run(void)
{
	for(int i = 0; i < task_num; i++)
	{
		uint32_t now_time = HAL_GetTick();
		if(now_time-scheduler_task[i].last_run >= scheduler_task[i].rate_ms)
		{
			scheduler_task[i].last_run = now_time;
			scheduler_task[i].task_func();
		}
	}
}
