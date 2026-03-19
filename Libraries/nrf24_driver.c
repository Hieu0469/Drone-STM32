#include "nrf24_driver.h"

SPI_HandleTypeDef *nrf_hspi;
GPIO_TypeDef *CE_PORT, *CSN_PORT;
uint16_t CE_PIN, CSN_PIN;

// Hàm hỗ trợ: Ghi lệnh vào nRF24
void NRF24_WriteReg(uint8_t reg, uint8_t data) {
    uint8_t buf[2];
    buf[0] = reg | W_REGISTER;
    buf[1] = data;
    HAL_GPIO_WritePin(CSN_PORT, CSN_PIN, GPIO_PIN_RESET);
    HAL_SPI_Transmit(nrf_hspi, buf, 2, 100);
    HAL_GPIO_WritePin(CSN_PORT, CSN_PIN, GPIO_PIN_SET);
}

// Khởi tạo cơ bản
void NRF24_Init(SPI_HandleTypeDef *hspi, GPIO_TypeDef *ce_port, uint16_t ce_pin, GPIO_TypeDef *csn_port, uint16_t csn_pin) {
    nrf_hspi = hspi; CE_PORT = ce_port; CE_PIN = ce_pin; CSN_PORT = csn_port; CSN_PIN = csn_pin;
    HAL_GPIO_WritePin(CE_PORT, CE_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(CSN_PORT, CSN_PIN, GPIO_PIN_SET);
}

// Chế độ TRUYỀN (Tx)
void NRF24_TxMode(uint8_t *Address, uint8_t Channel) {
    HAL_GPIO_WritePin(CE_PORT, CE_PIN, GPIO_PIN_RESET);
    NRF24_WriteReg(RF_CH, Channel); // Cài kênh
    // Ghi địa chỉ truyền
    HAL_GPIO_WritePin(CSN_PORT, CSN_PIN, GPIO_PIN_RESET);
    uint8_t cmd = TX_ADDR | W_REGISTER;
    HAL_SPI_Transmit(nrf_hspi, &cmd, 1, 100);
    HAL_SPI_Transmit(nrf_hspi, Address, 5, 100);
    HAL_GPIO_WritePin(CSN_PORT, CSN_PIN, GPIO_PIN_SET);
    
    // Cấu hình RF_SETUP: 250kbps, 0dBm (Với E01-2G4M27D nó sẽ tự khuếch đại lên 27dBm)
    NRF24_WriteReg(RF_SETUP, 0x26); 
    NRF24_WriteReg(CONFIG, 0x0A); // Power Up, Tx Mode
}

// Hàm gửi dữ liệu
uint8_t NRF24_Transmit(uint8_t *data) {
    uint8_t cmd = W_TX_PAYLOAD;
    HAL_GPIO_WritePin(CSN_PORT, CSN_PIN, GPIO_PIN_RESET);
    HAL_SPI_Transmit(nrf_hspi, &cmd, 1, 100);
    HAL_SPI_Transmit(nrf_hspi, data, 32, 100); // Gửi 32 bytes
    HAL_GPIO_WritePin(CSN_PORT, CSN_PIN, GPIO_PIN_SET);
    
    HAL_GPIO_WritePin(CE_PORT, CE_PIN, GPIO_PIN_SET); // Kích xung CE để gửi
    HAL_Delay(1);
    HAL_GPIO_WritePin(CE_PORT, CE_PIN, GPIO_PIN_RESET);
    return 1;
}
