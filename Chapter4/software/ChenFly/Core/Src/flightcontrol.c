#include "flightcontrol.h"
#include "pid.h"
#include "sbus.h"
#include "motor.h"
#include "icm20602.h"

extern uint16_t motorA_speed;
extern uint16_t motorB_speed;
extern uint16_t motorC_speed;
extern uint16_t motorD_speed;
extern uint8_t send_channels_data[21];
extern float roll, pitch, yaw;
float pidoutthrottle, pidoutroll, pidoutpitch, pidoutyaw;
float integral_roll, integral_pitch, integral_yaw;
float lasterr_roll, lasterr_pitch, lasterr_yaw;
float roll_P, roll_I, roll_D;
float pitch_P, pitch_I, pitch_D;
float yaw_P, yaw_I, yaw_D;
uint8_t motorstop_flag;
float desire_roll, desire_pitch, desire_yaw;

void init_flightcontrol(void)
{
	integral_roll = 0;
	integral_pitch = 0;
	integral_yaw = 0;
	lasterr_roll = 0;
	lasterr_pitch = 0;
	lasterr_yaw = 0;
	
	roll_P = 0.85f;
	roll_I = 0.003f;
	roll_D = 26.0f;
	pitch_P = 0.85f;
	pitch_I = 0.003f;
	pitch_D = 26.0f;
	yaw_P = 0.7f;
	yaw_I = 0.0f;
	yaw_D = 19.0f;
	
	motorstop_flag = 0;
	
	desire_roll = 0.0f;
	desire_pitch = 0.0f;
	desire_yaw = 0.0f;
}
void update_flightcontrol(void)
{
	pidoutthrottle = send_channels_data[6]*10+800;
	motorstop_flag = send_channels_data[8];
	desire_roll = ((float)send_channels_data[4]-50.0f)/2.5f;
	desire_pitch = ((float)send_channels_data[5]-50.0f)/2.5f;
	
	if(motorstop_flag > 70)
	{
		pidoutroll = update_pid((roll-desire_roll), roll_P, roll_I, roll_D, &integral_roll, 1500.0f, 300.0f, &lasterr_roll);
		pidoutpitch = update_pid((pitch+desire_pitch), pitch_P, pitch_I, pitch_D, &integral_pitch, 1500.0f, 300.0f, &lasterr_pitch);
		pidoutyaw = update_pid(yaw, yaw_P, yaw_I, yaw_D, &integral_yaw, 1500.0f, 300.0f, &lasterr_yaw);
		
		motorA_speed = pidoutthrottle + pidoutroll + pidoutpitch + pidoutyaw;
		motorB_speed = pidoutthrottle + pidoutroll - pidoutpitch - pidoutyaw;
		motorC_speed = pidoutthrottle - pidoutroll - pidoutpitch + pidoutyaw;
		motorD_speed = pidoutthrottle - pidoutroll + pidoutpitch - pidoutyaw;
	}
	else
	{
		motorA_speed = 1000;
		motorB_speed = 1000;
		motorC_speed = 1000;
		motorD_speed = 1000;
	}
}
