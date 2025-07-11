/**
 * @File : evse_rfid_8051.h
 * @Developed By : S GOPIKRISHNA
 * @Brief : Read RFID, RC522 Module through SPI Communication
 * @D.O.D : 08-07-2025
 */
#ifndef __EVSE_RFID_8051_H__
#define __EVSE_RFID_8051_H__

#include "ms51_16k_sdcc.h"

#define MAX_LEN        16
#define MI_OK          0
#define MI_NOTAGERR    1
#define MI_ERR         2

#define MFRC522_CS_LOW()     P15 = 0
#define MFRC522_CS_HIGH()    P15 = 1
#define MFRC522_RST_LOW()    P03 = 0
#define MFRC522_RST_HIGH()   P03 = 1

void init_rfid(void);
uint8_t read_rfid_tag(uint8_t *id);

#endif
