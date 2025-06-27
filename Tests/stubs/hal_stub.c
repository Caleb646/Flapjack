#include "hal.h"
#include <stdint.h>

HAL_StatusTypeDef
HAL_SPI_Transmit (SPI_HandleTypeDef* hspi, uint8_t* pData, uint16_t Size, uint32_t Timeout) {
}

HAL_StatusTypeDef
HAL_SPI_TransmitReceive (SPI_HandleTypeDef* hspi, uint8_t* pTxData, uint8_t* pRxData, uint16_t Size, uint32_t Timeout) {
}

HAL_StatusTypeDef
HAL_UART_Transmit (UART_HandleTypeDef* huart, const uint8_t* pData, uint16_t Size, uint32_t Timeout) {
}

uint32_t HAL_GetCurrentCPUID (void) {
}

void HAL_Delay (uint32_t ms) {
}

uint32_t HAL_GetTick (void) {
}

void __disable_irq (void) {
}
