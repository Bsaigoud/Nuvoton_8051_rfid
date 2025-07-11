/**
 * @File : evse_rs485_8051.h
 * @Developed By : S GOPIKRISHNA
 * @Brief : RS485 Transmitting Data
 * @D.O.D : 09-07-2025
 */
#ifndef __EVSE_RS485_8051_H__
#define __EVSE_RS485_8051_H__
#include "ms51_16k_sdcc.h"
#include <stdio.h>
#include <string.h>

struct SENDDATA {
    uint8_t device_id;
    uint8_t packet_type;
    char rfid_tag[10];
};
extern struct SENDDATA send_data;

void init_rs485(void);
void send_rfid_tag_info(struct SENDDATA *data);

#endif
