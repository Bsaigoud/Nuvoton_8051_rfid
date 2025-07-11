/**
 * @File : main.c
 * @Developed By : S GOPIKRISHNA
 * @Brief : Read RFID tag and send it through RS485
 * @D.O.D : 09-07-2025
 */

#include "ms51_16k_sdcc.h"
#include "evse_rfid_8051.h"
#include "evse_rs485_8051.h"

void main(void) {
    uint8_t uid[5];
    char tag[10] = {0};

    // Initialing System
    MODIFY_HIRC(HIRC_24);
    Enable_UART0_VCOM_printf_24M_115200();
    printf("\r\nSystem Started..\r\n");

    printf("\r\nInitializing RS485 Communication..\r\n");
    // Initializing RS485 Communication
    init_rs485();

    printf("\r\nInitializing RFID Module..\r\n");
    // Initializing RFID Module
    init_rfid();

    printf("\r\nTap Your RFID Card..\r\n");
    while (1) {
        for (uint8_t i = 0; i < 5; i++) uid[i] = 0;

        if (read_rfid_tag(uid) == MI_OK) {
            printf("\r\nRFID Card Detected: \r\n");
			sprintf(tag, "%02X%02X%02X%02X", uid[0], uid[1], uid[2], uid[3]);
			printf(" Tag id: %s\r\n", tag);
            printf("\r\n");

            memset(&send_data, 0, sizeof(send_data));
			send_data.device_id = 0x01;
			send_data.packet_type = 0x20;
			strncpy(send_data.rfid_tag, tag, sizeof(send_data.rfid_tag) - 1);
			send_data.rfid_tag[9] = '\0';

			send_rfid_tag_info(&send_data);
			UART_Send_Data(UART1, 'A');
            Timer0_Delay(24000000, 100, 1);  // 100ms delay between reads
        }
    }
}
