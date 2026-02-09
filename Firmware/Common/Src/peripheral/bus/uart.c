#include "peripheral/bus/uart.h"
#include "core/core.h"
#include "hal.h"
#include "mem/mem.h"
#include "peripheral/gpio.h"
#include "target.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define UART_CREATE(UART_NUM)                            \
    {                                                    \
        .id = UART_##UART_NUM##_ID, .hardware = {        \
            .pInstance = UART_##UART_NUM##_INSTANCE,     \
            .pRx       = UART_##UART_NUM##_RX_GPIO_PORT, \
            .pTx       = UART_##UART_NUM##_TX_GPIO_PORT, \
            .rxPin     = UART_##UART_NUM##_RX_GPIO_PIN,  \
            .txPin     = UART_##UART_NUM##_TX_GPIO_PIN,  \
            .af        = UART_##UART_NUM##_AF,           \
        }                                                \
    }


FJ_DEFINE_SHARED (Uart_t, g_Uarts[]) = {
#if defined(UART_1_ENABLED) && UART_1_ENABLED == 1U
    UART_CREATE (1),
#endif
};
FJ_DEFINE_SHARED (uint32_t, g_numUarts) = sizeof (g_Uarts) / sizeof (g_Uarts[0]);

eSTATUS_t Uart_InitSystem (void) {

    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = { 0 };

    PeriphClkInitStruct.PeriphClockSelection      = RCC_PERIPHCLK_USART1;
    PeriphClkInitStruct.Usart16ClockSelection     = RCC_USART16CLKSOURCE_D2PCLK2;
    PeriphClkInitStruct.Usart234578ClockSelection = 0U;
    if (HAL_RCCEx_PeriphCLKConfig (&PeriphClkInitStruct) != HAL_OK) {
        return eSTATUS_FAILURE;
    }
    __HAL_RCC_USART1_CLK_ENABLE ();

    PeriphClkInitStruct.PeriphClockSelection      = RCC_PERIPHCLK_USART2;
    PeriphClkInitStruct.Usart16ClockSelection     = 0U;
    PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig (&PeriphClkInitStruct) != HAL_OK) {
        return eSTATUS_FAILURE;
    }
    __HAL_RCC_USART2_CLK_ENABLE ();

    PeriphClkInitStruct.PeriphClockSelection      = RCC_PERIPHCLK_USART3;
    PeriphClkInitStruct.Usart16ClockSelection     = 0U;
    PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig (&PeriphClkInitStruct) != HAL_OK) {
        return eSTATUS_FAILURE;
    }
    __HAL_RCC_USART3_CLK_ENABLE ();

    for (uint32_t i = 0; i < g_numUarts; ++i) {

        Uart_t* pUart               = &g_Uarts[i];
        UartHardware_t* pHardware   = &pUart->hardware;
        UART_HandleTypeDef* pHandle = &pUart->handle;

        GPIO_InitTypeDef gpioInit = {
            .Mode      = GPIO_MODE_AF_PP,
            .Pull      = GPIO_NOPULL,
            .Speed     = GPIO_SPEED_FREQ_VERY_HIGH,
            .Alternate = pHardware->af,
        };
        HAL_GPIO_Init (pHardware->pRx, &gpioInit);
        HAL_GPIO_Init (pHardware->pTx, &gpioInit);

        pHandle->Instance                    = pHardware->pInstance;
        pHandle->Init.BaudRate               = 115200U;
        pHandle->Init.WordLength             = UART_WORDLENGTH_8B;
        pHandle->Init.StopBits               = UART_STOPBITS_1;
        pHandle->Init.Parity                 = UART_PARITY_NONE;
        pHandle->Init.Mode                   = UART_MODE_TX_RX;
        pHandle->Init.HwFlowCtl              = UART_HWCONTROL_NONE;
        pHandle->Init.OverSampling           = UART_OVERSAMPLING_16;
        pHandle->Init.OneBitSampling         = UART_ONE_BIT_SAMPLE_DISABLE;
        pHandle->Init.ClockPrescaler         = UART_PRESCALER_DIV1;
        pHandle->AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
        if (HAL_UART_Init (pHandle) != HAL_OK) {
            return eSTATUS_FAILURE;
        }

        if (HAL_UARTEx_SetTxFifoThreshold (pHandle, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK) {
            return eSTATUS_FAILURE;
        }

        if (HAL_UARTEx_SetRxFifoThreshold (pHandle, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK) {
            return eSTATUS_FAILURE;
        }
    }
    return eSTATUS_SUCCESS;
}

Uart_t* Uart_GetById (uart_id_t id) {

    for (uint32_t i = 0; i < g_numUarts; ++i) {
        if (g_Uarts[i].id == id) {
            return &g_Uarts[i];
        }
    }
    return NULL;
}

eSTATUS_t UartPort_Init (UartPort_t* pOutPort) {

    if (!pOutPort) {
        return eSTATUS_FAILURE;
    }
    Uart_t* pUart = Uart_GetById (pOutPort->cfg.id);
    if (!pUart) {
        return eSTATUS_FAILURE;
    }

    uint32_t baudRate = pOutPort->cfg.baudRate;
    if (baudRate != 0U) {

        __HAL_UART_DISABLE (&pUart->handle);
        pUart->handle.Init.BaudRate = baudRate;
        if (HAL_UART_Init (&pUart->handle) != HAL_OK) {
            return eSTATUS_FAILURE;
        }
        __HAL_UART_ENABLE (&pUart->handle);
    }

    pOutPort->pUart = pUart;
    return eSTATUS_SUCCESS;
}

eSTATUS_t UartPort_Write (UartPort_t* pPort, uint8_t const* pData, uint32_t size) {

    if (!pPort || !pData || size == 0U) {
        return eSTATUS_FAILURE;
    }
    if (HAL_UART_Transmit (&pPort->pUart->handle, (uint8_t*)pData, size, HAL_MAX_DELAY) != HAL_OK) {
        return eSTATUS_FAILURE;
    }
    return eSTATUS_SUCCESS;
}