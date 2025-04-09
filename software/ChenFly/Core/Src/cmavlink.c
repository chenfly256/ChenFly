#include "cmavlink.h"
#include "mavlink.h"
#include "usart.h"

#define PI          3.1415926f
mavlink_status_t status;
mavlink_message_t rec_mav_msg;
uint8_t chan = MAVLINK_COMM_0;
uint8_t system_id = 1;
uint8_t component_id = 1;
uint8_t target_system_id = 0;
uint8_t target_component_id = 0;
uint8_t type = MAV_TYPE_QUADROTOR;
uint8_t autopilot = MAV_AUTOPILOT_GENERIC;
uint8_t base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
uint8_t system_status = MAV_STATE_STANDBY;

extern float roll, pitch, yaw;

uint8_t send_flag = 0;
uint8_t send_param_flag = 0;
uint8_t send_param_index = 0;
uint8_t set_param_index = 0;
uint16_t len;
mavlink_message_t msg;
uint8_t pbuf[512] = {0x00};
char rec_msg_id[16];

mavlink_param_value_t param[PARAMCOUNT] = {
	{0.f, PARAMCOUNT, 0, "ROLL_P", MAV_PARAM_TYPE_REAL32},
	{0.f, PARAMCOUNT, 1, "ROLL_I", MAV_PARAM_TYPE_REAL32},
	{0.f, PARAMCOUNT, 2, "ROLL_D", MAV_PARAM_TYPE_REAL32},
	{0.f, PARAMCOUNT, 3, "PITCH_P", MAV_PARAM_TYPE_REAL32},
	{0.f, PARAMCOUNT, 4, "PITCH_I", MAV_PARAM_TYPE_REAL32},
	{0.f, PARAMCOUNT, 5, "PITCH_D", MAV_PARAM_TYPE_REAL32},
	{0.f, PARAMCOUNT, 6, "YAW_P", MAV_PARAM_TYPE_REAL32},
	{0.f, PARAMCOUNT, 7, "YAW_I", MAV_PARAM_TYPE_REAL32},
	{0.f, PARAMCOUNT, 8, "YAW_D", MAV_PARAM_TYPE_REAL32},
};

void send_mavlink(void)
{
	if(send_param_flag == 0)
	{
		if(send_flag > 19)
		{
			mavlink_msg_heartbeat_pack(system_id, component_id, &msg, type, autopilot, base_mode, 1, system_status);
			len = mavlink_msg_to_send_buffer(pbuf, &msg);
			HAL_UART_Transmit_DMA(&huart1, pbuf, len);
			
			send_flag = 0;
		}
		else if(send_flag == 1 || send_flag == 5 || send_flag == 9 || send_flag == 13  || send_flag == 17)
		{
			uint32_t now_time = HAL_GetTick();
			mavlink_msg_attitude_pack(system_id, component_id, &msg, now_time, roll*PI/180.f, pitch*PI/180.f, yaw*PI/180.f, 0.f, 0.f, 0.f);
			len = mavlink_msg_to_send_buffer(pbuf, &msg);
			HAL_UART_Transmit_DMA(&huart1, pbuf, len);
		}
	}
	else if(send_param_flag == 1)
	{
		mavlink_msg_param_value_pack(system_id, component_id, &msg, param[send_param_index].param_id, param[send_param_index].param_value, param[send_param_index].param_type, param[send_param_index].param_count, param[send_param_index].param_index);
		len = mavlink_msg_to_send_buffer(pbuf, &msg);
		HAL_UART_Transmit_DMA(&huart1, pbuf, len);
		
		send_param_index++;
		
		if(send_param_index == PARAMCOUNT)
		{
			send_param_index = 0;
			send_param_flag = 0;
		}
	}
	
	send_flag++;
}

void receive_mavlink_data(uint8_t *pbuf, uint8_t size)
{
	for(int i = 0; i < size; i++)
	{
		if (mavlink_parse_char(chan, pbuf[i], &rec_mav_msg, &status))
		{
			switch (rec_mav_msg.msgid)
			{
				case MAVLINK_MSG_ID_HEARTBEAT:
					target_system_id = rec_mav_msg.sysid;
					target_component_id = rec_mav_msg.compid;
					break;

				case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
					send_param_flag = 1;
					break;

				case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
					send_param_flag = 2;
					break;
				
				case MAVLINK_MSG_ID_PARAM_SET:
					mavlink_msg_param_set_get_param_id(&rec_mav_msg, rec_msg_id);
					for(int i = 0; i < PARAMCOUNT; i++)
					{
						if(strcmp(param[i].param_id, rec_msg_id) == 0)
						{
							param[i].param_value = mavlink_msg_param_set_get_param_value(&rec_mav_msg);
							set_param_index = i;
						}
					}
					send_param_flag = 1;
					break;
				
				default:
					
					break;
			}
		}
	}
}