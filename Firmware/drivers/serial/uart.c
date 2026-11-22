#include "hal.h"
#include "target.h"

#include "core/core.h"

#include "drivers/io/gpio.h"

#include "drivers/serial/uart.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define UART_CREATE(TYPE, UART_NUM) \
    {                                                    \
        .id       = UART_##UART_NUM##_ID,                \
        .hardware = {                                    \
            .pInstance = UART_##UART_NUM##_INSTANCE,     \
            .pRx       = UART_##UART_NUM##_RX_GPIO_PORT, \
            .pTx       = UART_##UART_NUM##_TX_GPIO_PORT, \
            .rxPin     = UART_##UART_NUM##_RX_GPIO_PIN,  \
            .txPin     = UART_##UART_NUM##_TX_GPIO_PIN,  \
            .af        = UART_##UART_NUM##_AF,           \
            .irqId     = TYPE##UART_NUM##_IRQn,       \
        },                                               \
                                                  \
    }

FJ_DEFINE_SHARED (Uart_t, g_Uarts[]) = {
#if BRD_IS_ENABLED(UART_1)
    UART_CREATE (USART, 1),
#endif
#if BRD_IS_ENABLED(UART_2)
    UART_CREATE (USART, 2),
#endif
#if BRD_IS_ENABLED(UART_3)
    UART_CREATE (USART, 3),
#endif
#if BRD_IS_ENABLED(UART_4)
    UART_CREATE (UART, 4),
#endif
#if BRD_IS_ENABLED(UART_5)
    UART_CREATE (UART, 5),
#endif
};
FJ_DEFINE_SHARED (uint32_t, g_NumUarts) = sizeof (g_Uarts) / sizeof (g_Uarts[0]);

void Uart_IrqHandler (uart_id_t id) {

    Uart_t* pUart = Uart_GetById (id);
    if (pUart) {
        USART_TypeDef* pInstance = pUart->hardware.pInstance;
        /* while, not if: with cfg.rxFifo set, several bytes can be waiting. */
        while (__HAL_UART_GET_FLAG (&pUart->handle, UART_FLAG_RXNE) != RESET && pUart->rxCallback) {
            uint8_t data = (uint8_t)(pInstance->RDR);
            pUart->rxCallback (&data, 1U);
        }
        /*
         * HAL treats an overrun as a blocking error: HAL_UART_IRQHandler clears
         * RXNEIE and nothing re-enables it, so one late byte used to silence the
         * port until reboot - on the RC link, a lost link that never recovers.
         * Clearing it first keeps HAL off that path; the dropped byte costs one
         * frame its CRC. RDR was just read, so another overrun needs two more
         * byte-times, far longer than HAL takes to sample the flags.
         */
        if (pUart->rxCallback && __HAL_UART_GET_FLAG (&pUart->handle, UART_FLAG_ORE) != RESET) {
            __HAL_UART_CLEAR_FLAG (&pUart->handle, UART_CLEAR_OREF);
        }
        HAL_UART_IRQHandler (&pUart->handle);
    }
}

#define UART_DEF_ISR(TYPE, UART_NUM)            \
    void TYPE##UART_NUM##_IRQHandler (void) {   \
        Uart_IrqHandler (UART_##UART_NUM##_ID); \
    }

#if BRD_IS_ENABLED(UART_1)
UART_DEF_ISR (USART, 1)
#endif
#if BRD_IS_ENABLED(UART_2)
UART_DEF_ISR (USART, 2)
#endif
#if BRD_IS_ENABLED(UART_3)
UART_DEF_ISR (USART, 3)
#endif
#if BRD_IS_ENABLED(UART_4)
UART_DEF_ISR (UART, 4)
#endif
#if BRD_IS_ENABLED(UART_5)
UART_DEF_ISR (UART, 5)
#endif

eSTATUS_t Uart_InitSystem (void) {

    /* Kernel clock sources are selected in Platform_Init. */
    __HAL_RCC_USART1_CLK_ENABLE ();
    __HAL_RCC_USART2_CLK_ENABLE ();
    __HAL_RCC_USART3_CLK_ENABLE ();
    __HAL_RCC_UART4_CLK_ENABLE ();
    __HAL_RCC_UART5_CLK_ENABLE ();


    // HAL_NVIC_SetPriority (USART1_IRQn, 5, 0);
    // HAL_NVIC_EnableIRQ (USART1_IRQn);
    // __HAL_UART_ENABLE_IT (&g_Uarts[0].handle, UART_IT_ERR);
    // __HAL_UART_ENABLE_IT (&g_Uarts[0].handle, UART_IT_NE);
    //__HAL_UART_ENABLE_IT (&g_Uarts[0]->handle, UART_IT_);


    for (uint32_t i = 0; i < g_NumUarts; ++i) {

        Uart_t* pUart               = &g_Uarts[i];
        UartHardware_t* pHardware   = &pUart->hardware;
        UART_HandleTypeDef* pHandle = &pUart->handle;

        GPIO_ENABLE_CLOCK (pHardware->pRx);
        GPIO_ENABLE_CLOCK (pHardware->pTx);

        GPIO_InitTypeDef gpioInit = {
            .Mode      = GPIO_MODE_AF_PP,
            .Pull      = GPIO_NOPULL,
            .Speed     = GPIO_SPEED_FREQ_VERY_HIGH,
            .Alternate = pHardware->af,
        };

        gpioInit.Pin = pHardware->rxPin;
        HAL_GPIO_Init (pHardware->pRx, &gpioInit);

        gpioInit.Pin = pHardware->txPin;
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

        if (HAL_UARTEx_DisableFifoMode (pHandle) != HAL_OK) {
            return eSTATUS_FAILURE;
        }
    }
    return eSTATUS_SUCCESS;
}

Uart_t* Uart_GetById (uart_id_t id) {

    for (uint32_t i = 0; i < g_NumUarts; ++i) {
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
    if (baudRate) {

        __HAL_UART_DISABLE (&pUart->handle);
        pUart->handle.Init.BaudRate = baudRate;
        if (HAL_UART_Init (&pUart->handle) != HAL_OK) {
            return eSTATUS_FAILURE;
        }
        __HAL_UART_ENABLE (&pUart->handle);
    }

    /* FIFOEN repurposes RXNEIE as FIFO-not-empty, so the RXNE interrupt
     * enabled below drives the FIFO with no other change. */
    if (pOutPort->cfg.rxFifo && HAL_UARTEx_EnableFifoMode (&pUart->handle) != HAL_OK) {
        return eSTATUS_FAILURE;
    }

    if (pOutPort->cfg.rxCallback) {
        HAL_NVIC_SetPriority (pUart->hardware.irqId, pOutPort->cfg.irqPriority, 0);
        HAL_NVIC_EnableIRQ (pUart->hardware.irqId);
        pUart->rxCallback = pOutPort->cfg.rxCallback;
        __HAL_UART_ENABLE_IT (&pUart->handle, UART_IT_RXNE);
    }

    pOutPort->pUart = pUart;
    return eSTATUS_SUCCESS;
}

eSTATUS_t UartPort_Write (UartPort_t* pPort, uint8_t const* pData, uint32_t size) {

    if (!pPort || !pData || !size) {
        return eSTATUS_FAILURE;
    }
    if (HAL_UART_Transmit (&pPort->pUart->handle, pData, size, HAL_MAX_DELAY) != HAL_OK) {
        return eSTATUS_FAILURE;
    }
    return eSTATUS_SUCCESS;
}