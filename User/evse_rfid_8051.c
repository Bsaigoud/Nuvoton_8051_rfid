/**
 * @File : evse_rfid_8051.c
 * @Developed By : S GOPIKRISHNA
 * @Brief : Read RFID, RC522 Module through SPI Communication
 * @D.O.D : 08-07-2025
 */
#include "evse_rfid_8051.h"
#include "numicro_8051.h"
#include <string.h>

#define PCD_IDLE              0x00
#define PCD_TRANSCEIVE        0x0C
#define PCD_RESETPHASE        0x0F

#define PICC_REQIDL           0x26
#define PICC_ANTICOLL         0x93

#define CommandReg            0x01
#define CommIEnReg            0x02
#define CommIrqReg            0x04
#define DivIrqReg             0x05
#define ErrorReg              0x06
#define FIFODataReg           0x09
#define FIFOLevelReg          0x0A
#define ControlReg            0x0C
#define BitFramingReg         0x0D
#define ModeReg               0x11
#define TxControlReg          0x14
#define TxASKReg              0x15
#define CRCResultRegH         0x21
#define CRCResultRegL         0x22
#define TModeReg              0x2A
#define TPrescalerReg         0x2B
#define TReloadRegH           0x2C
#define TReloadRegL           0x2D

void SPI0_Init(void) {
    P10_QUASI_MODE; // SCK
    P00_QUASI_MODE; // MOSI
    P01_QUASI_MODE; // MISO
    P15_QUASI_MODE; // SS
    P03_QUASI_MODE; // RST

    set_SPSR_DISMODF;
    clr_SPCR_LSBFE;
    clr_SPCR_CPOL;
    clr_SPCR_CPHA;
    set_SPCR_MSTR;
    set_SPCR_SPR1; clr_SPCR_SPR0;  // SPI clock = Fsys/64
    set_SPCR_SPIEN;
}

uint8_t SPI_WriteByte(uint8_t byte) {
    SPDR = byte;
    while (!(SPSR & 0x80));
    clr_SPSR_SPIF;
    return SPDR;
}

void MFRC522_WriteReg(uint8_t addr, uint8_t val) {
    MFRC522_CS_LOW();
    SPI_WriteByte((addr << 1) & 0x7E);
    SPI_WriteByte(val);
    MFRC522_CS_HIGH();
}

uint8_t MFRC522_ReadReg(uint8_t addr) {
    uint8_t val;
    MFRC522_CS_LOW();
    SPI_WriteByte(((addr << 1) & 0x7E) | 0x80);
    val = SPI_WriteByte(0x00);
    MFRC522_CS_HIGH();
    return val;
}

void MFRC522_SetBitMask(uint8_t reg, uint8_t mask) {
    MFRC522_WriteReg(reg, MFRC522_ReadReg(reg) | mask);
}

void MFRC522_ClearBitMask(uint8_t reg, uint8_t mask) {
    MFRC522_WriteReg(reg, MFRC522_ReadReg(reg) & (~mask));
}

void MFRC522_AntennaOn(void) {
    if (!(MFRC522_ReadReg(TxControlReg) & 0x03))
        MFRC522_SetBitMask(TxControlReg, 0x03);
}

void MFRC522_Reset(void) {
    MFRC522_RST_LOW();
    Timer0_Delay(24000000, 1, 1);	// 1ms
    MFRC522_RST_HIGH();
    Timer0_Delay(24000000, 10, 1);
    MFRC522_WriteReg(CommandReg, PCD_RESETPHASE);
}

void init_rfid(void) {

	//Initializing SPI BUS
	SPI0_Init();

	// Initializing RFID
    MFRC522_CS_HIGH();
    MFRC522_Reset();

    MFRC522_WriteReg(TModeReg, 0x8D);
    MFRC522_WriteReg(TPrescalerReg, 0x3E);
    MFRC522_WriteReg(TReloadRegL, 30);
    MFRC522_WriteReg(TReloadRegH, 0);
    MFRC522_WriteReg(TxASKReg, 0x40);
    MFRC522_WriteReg(ModeReg, 0x3D);
    MFRC522_AntennaOn();
}

uint8_t MFRC522_ToCard(uint8_t command, uint8_t *sendData, uint8_t sendLen, uint8_t *backData, uint16_t *backLen) {
    uint8_t status = MI_ERR;
    uint8_t irqEn = 0x77;
    uint8_t waitIRq = 0x30;
    uint8_t n, lastBits;
    uint16_t i;

    MFRC522_WriteReg(CommIEnReg, irqEn | 0x80);
    MFRC522_ClearBitMask(CommIrqReg, 0x80);
    MFRC522_SetBitMask(FIFOLevelReg, 0x80);
    MFRC522_WriteReg(CommandReg, PCD_IDLE);

    for (i = 0; i < sendLen; i++) MFRC522_WriteReg(FIFODataReg, sendData[i]);
    MFRC522_WriteReg(CommandReg, command);
    if (command == PCD_TRANSCEIVE) MFRC522_SetBitMask(BitFramingReg, 0x80);

    i = 2000;
    do {
        n = MFRC522_ReadReg(CommIrqReg);
    } while (!(n & 0x01) && !(n & waitIRq) && --i);

    MFRC522_ClearBitMask(BitFramingReg, 0x80);
    if (i && !(MFRC522_ReadReg(ErrorReg) & 0x1B)) {
        status = MI_OK;
        if (command == PCD_TRANSCEIVE) {
            n = MFRC522_ReadReg(FIFOLevelReg);
            lastBits = MFRC522_ReadReg(ControlReg) & 0x07;
            *backLen = lastBits ? (n - 1) * 8 + lastBits : n * 8;
            for (i = 0; i < n; i++) backData[i] = MFRC522_ReadReg(FIFODataReg);
        }
    }
    return status;
}

uint8_t MFRC522_Request(uint8_t reqMode, uint8_t *tagType) {
    uint8_t status;
    uint16_t backBits;
    MFRC522_WriteReg(BitFramingReg, 0x07);
    tagType[0] = reqMode;
    status = MFRC522_ToCard(PCD_TRANSCEIVE, tagType, 1, tagType, &backBits);
//    return (status == MI_OK && backBits == 0x10) ? MI_OK : MI_ERR;
    if ((status != MI_OK) || (backBits != 0x10))
            status = MI_ERR;
    return status;
}

uint8_t MFRC522_Anticoll(uint8_t *serNum) {
    uint8_t status, i, serNumCheck = 0;
    uint16_t unLen;
    MFRC522_WriteReg(BitFramingReg, 0x00);
    serNum[0] = PICC_ANTICOLL; serNum[1] = 0x20;
    status = MFRC522_ToCard(PCD_TRANSCEIVE, serNum, 2, serNum, &unLen);
    if (status == MI_OK) {
        for (i = 0; i < 4; i++) serNumCheck ^= serNum[i];
        if (serNumCheck != serNum[4]) status = MI_ERR;
    }
    return status;
}

void MFRC522_CalculateCRC(uint8_t *inData, uint8_t len, uint8_t *outData) {
    uint8_t i;
    MFRC522_WriteReg(CommandReg, PCD_IDLE);
    MFRC522_SetBitMask(DivIrqReg, 0x04);
    MFRC522_SetBitMask(FIFOLevelReg, 0x80);
    for (i = 0; i < len; i++) MFRC522_WriteReg(FIFODataReg, inData[i]);
    MFRC522_WriteReg(CommandReg, 0x03);  // PCD_CALCCRC
    i = 255;
    while (!(MFRC522_ReadReg(DivIrqReg) & 0x04)) {
        if (--i == 0) break;
    }
    outData[0] = MFRC522_ReadReg(CRCResultRegL);
    outData[1] = MFRC522_ReadReg(CRCResultRegH);
}

void TM_MFRC522_Halt(void) {
    uint8_t buffer[4];
    uint16_t backLen;
    buffer[0] = 0x50;
    buffer[1] = 0x00;
    MFRC522_CalculateCRC(buffer, 2, &buffer[2]);
    MFRC522_ToCard(PCD_TRANSCEIVE, buffer, 4, buffer, &backLen);
}

uint8_t TM_MFRC522_SelectCard(uint8_t *serNum) {
    uint8_t status;
    uint8_t buffer[9];
    uint16_t recvBits;
    uint8_t sak = 0;
    buffer[0] = 0x93;
    buffer[1] = 0x70;
    for (uint8_t i = 0; i < 5; i++) buffer[i + 2] = serNum[i];
    MFRC522_CalculateCRC(buffer, 7, &buffer[7]);
    status = MFRC522_ToCard(PCD_TRANSCEIVE, buffer, 9, buffer, &recvBits);
    if (status == MI_OK && recvBits == 0x18)  // Expecting 3 bytes (24 bits)
       {
           sak = buffer[0];  // SAK is in buffer[0]
       }

       return sak; // or return status if preferred
//    return (status == MI_OK && (recvBits == 0x18)) ? MI_OK : MI_ERR;
}

uint8_t read_rfid_tag(uint8_t *id) {
	uint8_t status;
	status = MFRC522_Request(PICC_REQIDL, id);

	if (status == MI_OK)
	status = MFRC522_Anticoll(id);

	if (status == MI_OK)
	{
		if (TM_MFRC522_SelectCard(id) == MI_OK)
		{
			if (id[0] & 0x04)
				printf("UID not complete.\r\n");
		}
		TM_MFRC522_Halt();
	}
	else
		status = MI_ERR;
	return status;
}
