#ifndef HAL_H
#define HAL_H

#ifdef UNIT_TEST

// #include "../Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_hal_dma.h"
#include "stm32h7xx_hal_dma.h"
#include <stdint.h>

#define __BKPT(arg)
#define __NOP()
#define CM7_CPUID ((uint32_t)0x00000003)
#define CM4_CPUID ((uint32_t)0x00000001)

// clang-format off

typedef enum {
    HAL_OK      = 0x00,
    HAL_ERROR   = 0x01,
    HAL_BUSY    = 0x02,
    HAL_TIMEOUT = 0x03
} HAL_StatusTypeDef;

void HAL_Delay (uint32_t ms);
uint32_t HAL_GetTick (void);
uint32_t HAL_GetCurrentCPUID (void);
void __disable_irq (void);

// typedef struct {
//     int dummy;
// } DMA_HandleTypeDef;

// typedef struct
// {
//   uint32_t CR;  
//   uint32_t NDTR;
//   uint32_t PAR; 
//   uint32_t M0AR;
//   uint32_t M1AR;
//   uint32_t FCR; 
// } DMA_Stream_TypeDef;

// extern DMA_Stream_TypeDef g_DMA1_Stream0;
// extern DMA_Stream_TypeDef g_DMA1_Stream1;
// extern DMA_Stream_TypeDef g_DMA1_Stream2;
// extern DMA_Stream_TypeDef g_DMA1_Stream3;
// extern DMA_Stream_TypeDef g_DMA1_Stream4;
// extern DMA_Stream_TypeDef g_DMA1_Stream5;
// extern DMA_Stream_TypeDef g_DMA1_Stream6;
// extern DMA_Stream_TypeDef g_DMA1_Stream7;
// #define DMA1_Stream0 (&g_DMA1_Stream0)
// #define DMA1_Stream1 (&g_DMA1_Stream1)
// #define DMA1_Stream2 (&g_DMA1_Stream2)
// #define DMA1_Stream3 (&g_DMA1_Stream3)
// #define DMA1_Stream4 (&g_DMA1_Stream4)
// #define DMA1_Stream5 (&g_DMA1_Stream5)
// #define DMA1_Stream6 (&g_DMA1_Stream6)
// #define DMA1_Stream7 (&g_DMA1_Stream7)

// void HAL_DMA_IRQHandler(DMA_HandleTypeDef *hdma);

typedef struct {
    int dummy;
} SPI_HandleTypeDef;

typedef struct {
    int dummy;
} UART_HandleTypeDef;

typedef struct {
    int dummy;
} USART_TypeDef;

HAL_StatusTypeDef HAL_SPI_Init (SPI_HandleTypeDef* hspi);
HAL_StatusTypeDef HAL_SPI_DeInit (SPI_HandleTypeDef* hspi);
// HAL_StatusTypeDef HAL_SPI_Receive
HAL_StatusTypeDef HAL_SPI_Transmit (SPI_HandleTypeDef* hspi, uint8_t* pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_SPI_TransmitReceive (SPI_HandleTypeDef* hspi, uint8_t* pTxData, uint8_t* pRxData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_UART_Transmit (UART_HandleTypeDef* huart, const uint8_t* pData, uint16_t Size, uint32_t Timeout);

typedef HAL_StatusTypeDef (*HAL_SPI_Transmit_CB) (SPI_HandleTypeDef* hspi, uint8_t* pData, uint16_t Size, uint32_t Timeout);
typedef HAL_StatusTypeDef (*HAL_SPI_TransmitReceive_CB) (SPI_HandleTypeDef* hspi, uint8_t* pTxData, uint8_t* pRxData, uint16_t Size, uint32_t Timeout);
typedef HAL_StatusTypeDef (*HAL_UART_Transmit_CB) (UART_HandleTypeDef* huart, const uint8_t* pData, uint16_t Size, uint32_t Timeout);

extern HAL_SPI_Transmit_CB gHAL_SPI_Transmit_CB;
extern HAL_SPI_TransmitReceive_CB gHAL_SPI_TransmitReceive_CB;
extern HAL_UART_Transmit_CB gHAL_UART_Transmit_CB;


// clang-format on
// #ifdef UNIT_TEST

#else

#include "cmsis_os.h"
#include "stm32h7xx.h"
#include "stm32h7xx_hal.h"

#endif

#endif // HAL_H