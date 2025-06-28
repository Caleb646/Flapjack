#include "hal.h"
#include <stdint.h>
#include <string.h>

extern HAL_SPI_Transmit_CB gHAL_SPI_Transmit_CB               = NULL;
extern HAL_SPI_TransmitReceive_CB gHAL_SPI_TransmitReceive_CB = NULL;
extern HAL_UART_Transmit_CB gHAL_UART_Transmit_CB             = NULL;

HAL_StatusTypeDef
HAL_SPI_Transmit (SPI_HandleTypeDef* hspi, uint8_t* pData, uint16_t Size, uint32_t Timeout) {
    if (gHAL_SPI_Transmit_CB) {
        return gHAL_SPI_Transmit_CB (hspi, pData, Size, Timeout);
    }
    return HAL_OK; // or your default stub behavior
}

HAL_StatusTypeDef
HAL_SPI_TransmitReceive (SPI_HandleTypeDef* hspi, uint8_t* pTxData, uint8_t* pRxData, uint16_t Size, uint32_t Timeout) {
    if (gHAL_SPI_TransmitReceive_CB) {
        return gHAL_SPI_TransmitReceive_CB (hspi, pTxData, pRxData, Size, Timeout);
    }
    return HAL_OK; // or your default stub behavior
}

HAL_StatusTypeDef
HAL_UART_Transmit (UART_HandleTypeDef* huart, const uint8_t* pData, uint16_t Size, uint32_t Timeout) {
    if (gHAL_UART_Transmit_CB) {
        return gHAL_UART_Transmit_CB (huart, pData, Size, Timeout);
    }
    return HAL_OK; // or your default stub behavior
}

uint32_t HAL_GetCurrentCPUID (void) {
}

void HAL_Delay (uint32_t ms) {
}

uint32_t HAL_GetTick (void) {
}

void __disable_irq (void) {
}
