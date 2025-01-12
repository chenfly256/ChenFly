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

float pidoutthrottle, pidoutroll, pidoutpitch, pidoutyaw;
float integral_roll, integral_pitch, integral_yaw;
float roll_P, roll_I, roll_D;
float pitch_P, pitch_I, pitch_D;
float yaw_P, yaw_I, yaw_D;

uint8_t motorstop_flag;

extern float roll, pitch, yaw;
extern float roll_rate, pitch_rate, yaw_rate;
float lasterr_roll_rate, lasterr_pitch_rate, lasterr_yaw_rate;
float desire_roll_rate, desire_pitch_rate, desire_yaw_rate;
float desire_roll_angle, desire_pitch_angle, desire_yaw_angle;

void init_flightcontrol(void)
{
	integral_roll = 0;
	integral_pitch = 0;
	integral_yaw = 0;
	lasterr_roll_rate = 0;
	lasterr_pitch_rate = 0;
	lasterr_yaw_rate = 0;
	
	roll_P = 1.2f;
	roll_I = 0.01f;
	roll_D = 8.0f;
	pitch_P = 1.2f;
	pitch_I = 0.01f;
	pitch_D = 8.0f;
	yaw_P = 1.6f;
	yaw_I = 0.0f;
	yaw_D = 0.0f;
	
	motorstop_flag = 0;
	
	desire_roll_rate = 0.0f;
	desire_pitch_rate = 0.0f;
	desire_yaw_rate = 0.0f;
	desire_roll_angle = 0.0f;
	desire_pitch_angle = 0.0f;
	desire_yaw_angle = 0.0f;
}
void update_flightcontrol(void)
{
	pidoutthrottle = send_channels_data[6]*10+800;
	motorstop_flag = send_channels_data[8];
	
	desire_roll_angle = ((float)send_channels_data[4]-50.0f)/2.5f;
	desire_pitch_angle = ((float)send_channels_data[5]-50.0f)/2.5f;
	
	desire_roll_rate = (desire_roll_angle-roll)*0.6f;
	if(desire_roll_rate > 45.0f)
		desire_roll_rate = 45.0f;
	else if(desire_roll_rate < -45.0f)
		desire_roll_rate = -45.0f;
	desire_pitch_rate = (desire_pitch_angle-pitch)*0.6f;
	if(desire_pitch_rate > 45.0f)
		desire_pitch_rate = 45.0f;
	else if(desire_pitch_rate < -45.0f)
		desire_pitch_rate = -45.0f;
	desire_yaw_rate = -((float)send_channels_data[7]-50.0f)/2.0f;
	
	if(motorstop_flag > 70)
	{
		pidoutroll = update_pid(roll_rate-desire_roll_rate, roll_P, roll_I, roll_D, &integral_roll, 1500.0f, 300.0f, &lasterr_roll_rate);
		pidoutpitch = update_pid(pitch_rate-desire_pitch_rate, pitch_P, pitch_I, pitch_D, &integral_pitch, 1500.0f, 300.0f, &lasterr_pitch_rate);
		pidoutyaw = update_pid(yaw_rate-desire_yaw_rate, yaw_P, yaw_I, yaw_D, &integral_yaw, 1500.0f, 300.0f, &lasterr_yaw_rate);
		
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
