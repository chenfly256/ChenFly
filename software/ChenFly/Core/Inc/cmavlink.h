#ifndef _CMAVLINK_H_
#define _CMAVLINK_H_

#include "stm32f4xx.h"

void send_mavlink(void);
void receive_mavlink_data(uint8_t *pbuf, uint8_t size);

#define PARAMCOUNT 9

#endif