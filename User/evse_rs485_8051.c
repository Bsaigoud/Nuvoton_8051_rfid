/**
 * @File : evse_rs485_8051.c
 * @Developed By : S GOPIKRISHNA
 * @Brief : RS485 Transmitting Data
 * @D.O.D : 09-07-2025
 */
#include "evse_rs485_8051.h"
#include "ms51_16k_sdcc.h"

struct SENDDATA send_data;

void init_rs485(void)
{
    P16_QUASI_MODE;
    P02_INPUT_MODE;
    UART_Open(24000000,UART1_Timer3,115200);
}

void send_rfid_tag_info(struct SENDDATA *data)
{
    uint8_t *ptr = (uint8_t *)data;
    for (uint8_t i = 0; i < sizeof(struct SENDDATA); i++)
    {
        UART_Send_Data(UART1, ptr[i]);
//        Timer0_Delay(24000000, 1, 300);
    }
}
