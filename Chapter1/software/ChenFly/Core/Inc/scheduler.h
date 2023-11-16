#ifndef _SCHEDULER_H_
#define _SCHEDULER_H_

#include "stm32f4xx.h"

typedef struct
{
	void(*task_func)(void);
	uint16_t rate_ms;
	uint32_t last_run;
}scheduler_task_t;
void Scheduler_init(void);
void Scheduler_run(void);

#endif
