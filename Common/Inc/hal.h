#ifndef HAL_H
#define HAL_H

#ifdef UNIT_TEST

#include <stdint.h>

#define __BKPT(arg)
#define CM7_CPUID ((uint32_t)0x00000003)
#define CM4_CPUID ((uint32_t)0x00000001)

typedef struct {
    int dummy;
} SPI_HandleTypeDef;
typedef struct {
    int dummy;
} UART_HandleTypeDef;
typedef enum {
    HAL_OK      = 0x00,
    HAL_ERROR   = 0x01,
    HAL_BUSY    = 0x02,
    HAL_TIMEOUT = 0x03
} HAL_StatusTypeDef;


HAL_StatusTypeDef
HAL_SPI_Transmit (SPI_HandleTypeDef* hspi, uint8_t* pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef
HAL_SPI_TransmitReceive (SPI_HandleTypeDef* hspi, uint8_t* pTxData, uint8_t* pRxData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef
HAL_UART_Transmit (UART_HandleTypeDef* huart, const uint8_t* pData, uint16_t Size, uint32_t Timeout);
void HAL_Delay (uint32_t ms);
uint32_t HAL_GetTick (void);
uint32_t HAL_GetCurrentCPUID (void);
void __disable_irq (void);

#else

#include "cmsis_os.h"
#include "stm32h7xx.h"
#include "stm32h7xx_hal.h"

#endif

#endif // HAL_H