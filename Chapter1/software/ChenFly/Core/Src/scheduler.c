#include "scheduler.h"
#include "usart.h"
#include "gpio.h"

uint8_t task_num;
uint8_t uart1_test_data[] = {0xFE, 0x01, 0x02, 0x03};

static void led_blink(void)
{
	if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == GPIO_PIN_RESET)
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
}
static void uart1_test(void)
{
	HAL_UART_Transmit(&huart1, uart1_test_data, 4, 0xffff);
}
static scheduler_task_t scheduler_task[] =
{
	{led_blink, 500, 0},
	{uart1_test, 100, 0}
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
