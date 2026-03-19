#ifndef NRF24_DRIVER_H
#define NRF24_DRIVER_H

#include "stm32f1xx_hal.h" // Thay bằng dòng chip của bạn (f4, f1,...)

// Địa chỉ các thanh ghi (Registers)
#define CONFIG      0x00
#define EN_AA       0x01
#define EN_RXADDR   0x02
#define SETUP_AW    0x03
#define SETUP_RETR  0x04
#define RF_CH       0x05
#define RF_SETUP    0x06
#define STATUS      0x07
#define RX_ADDR_P0  0x0A
#define TX_ADDR     0x10
#define RX_PW_P0    0x11
#define FIFO_STATUS 0x17

// Lệnh (Commands)
#define R_REGISTER    0x00
#define W_REGISTER    0x20
#define R_RX_PAYLOAD  0x61
#define W_TX_PAYLOAD  0xA0
#define FLUSH_TX      0xE1
#define FLUSH_RX      0xE2

// Các hàm cơ bản
void NRF24_Init(SPI_HandleTypeDef *hspi, GPIO_TypeDef *ce_port, uint16_t ce_pin, GPIO_TypeDef *csn_port, uint16_t csn_pin);
void NRF24_TxMode(uint8_t *Address, uint8_t Channel);
void NRF24_RxMode(uint8_t *Address, uint8_t Channel);
uint8_t NRF24_Transmit(uint8_t *data);
uint8_t NRF24_DataAvailable(void);
void NRF24_Receive(uint8_t *data);

#endif
