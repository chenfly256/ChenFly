#include "motor.h"
#include "tim.h"

uint16_t motorA_speed;
uint16_t motorB_speed;
uint16_t motorC_speed;
uint16_t motorD_speed;

void motor_init(void)
{
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 2000);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 2000);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, 2000);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, 2000);
	HAL_Delay(2000);
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 1000);
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 1000);
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, 1000);
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, 1000);
	HAL_Delay(2000);
	motorA_speed = 1000;
	motorB_speed = 1000;
	motorC_speed = 1000;
	motorD_speed = 1000;
}

void motor_speed_update(void)
{
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, motorA_speed);
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, motorB_speed);
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, motorC_speed);
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, motorD_speed);
}
