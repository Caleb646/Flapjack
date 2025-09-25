#include "peripheral/uart.h"
#include "common.h"
#include "hal.h"
#include "mem/mem.h"
#include "peripheral/gpio.h"
#include <stdint.h>
#include <string.h>

#define IS_BUS_VALID(pBUS) \
    ((pBUS) != NULL && (pBUS)->isInitialized == TRUE)

static SHARED_MEM_SECTION UARTBus_t gBuses[eUART_BUS_ID_MAX] = { 0 };

static vUARTBus_t* UARTGetBusById (eBUS_ID_t busId) {

    uint32_t busIndex = UART_BUS_ID2IDX (busId);
    if (BUS_ID_IS_UART (busId) == FALSE || busIndex >= eUART_BUS_ID_MAX) {
        return NULL;
    }
    return &gBuses[busIndex];
}

static USART_TypeDef* UARTGetInstanceById (eBUS_ID_t busId) {

    switch (busId) {
    case eUART_1_BUS_ID: return USART1;
    case eUART_2_BUS_ID: return USART2;
    case eUART_3_BUS_ID: return USART3;
    default: return NULL;
    }
}

static vUARTBus_t* UARTGetBusByInstance (USART_TypeDef* instance) {

    for (uint32_t i = 0; i < eUART_BUS_ID_MAX; i++) {
        if (gBuses[i].handle.Instance == instance) {
            return &gBuses[i];
        }
    }
    return NULL;
}

/*
 * Below are functions declared by the HAL and defined here.
 */

void USART1_IRQHandler (void) {
    HAL_UART_IRQHandler (&UARTGetBusById (eUART_1_BUS_ID)->handle);
}

void USART2_IRQHandler (void) {
    HAL_UART_IRQHandler (&UARTGetBusById (eUART_2_BUS_ID)->handle);
}

void USART3_IRQHandler (void) {
    HAL_UART_IRQHandler (&UARTGetBusById (eUART_3_BUS_ID)->handle);
}

void HAL_UART_TxCpltCallback (UART_HandleTypeDef* huart) {

    vUARTBus_t* pBus = UARTGetBusByInstance (huart->Instance);
    if (IS_BUS_VALID (pBus) == FALSE) {
        return;
    }
    if (pBus->txCallback != NULL) {
        pBus->txCallback (pBus->busId);
    }
}

void HAL_UART_RxCpltCallback (UART_HandleTypeDef* huart) {

    vUARTBus_t* pBus = UARTGetBusByInstance (huart->Instance);
    if (IS_BUS_VALID (pBus) == FALSE) {
        return;
    }
    if (pBus->rxCallback != NULL) {
        pBus->rxCallback (pBus->busId);
    }
}

void HAL_UART_ErrorCallback (UART_HandleTypeDef* huart) {

    vUARTBus_t* pBus = UARTGetBusByInstance (huart->Instance);
    if (IS_BUS_VALID (pBus) == FALSE) {
        return;
    }
    if (pBus->errorCallback != NULL) {
        pBus->errorCallback (pBus->busId);
    }
}

/*
 * Called by HAL_UART_Init()
 */
// void HAL_UART_MspInit (UART_HandleTypeDef* huart) {

//     RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = { 0 };
//     if (huart->Instance == USART1) {
//         /*
//          *  Initializes the peripherals clock
//          */
//         PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART1;
//         PeriphClkInitStruct.Usart16ClockSelection = RCC_USART16CLKSOURCE_D2PCLK2;
//         if (HAL_RCCEx_PeriphCLKConfig (&PeriphClkInitStruct) != HAL_OK) {
//             CriticalErrorHandler ();
//         }
//         /* Peripheral clock enable */
//         __HAL_RCC_USART1_CLK_ENABLE ();
//     }
// }

// void HAL_UART_MspDeInit (UART_HandleTypeDef* huart) {
//     if (huart->Instance == USART1) {
//         /* Peripheral clock disable */
//         __HAL_RCC_USART1_CLK_DISABLE ();

//         /**USART1 GPIO Configuration
//         PA10     ------> USART1_RX
//         PA9     ------> USART1_TX
//         */
//         // HAL_GPIO_DeInit (GPIOA, STLINK_TX_GPIO_Pin | STLINK_RX_GPIO_Pin);
//     }
// }

#define UART_CLOCK_INIT(pSTATUS, BUS_ID, PERIPHCLK, CLKSELECTION)         \
    do {                                                                  \
        RCC_PeriphCLKInitTypeDef PeriphClkInitStruct  = { 0 };            \
        PeriphClkInitStruct.PeriphClockSelection      = PERIPHCLK;        \
        PeriphClkInitStruct.Usart234578ClockSelection = CLKSELECTION;     \
        if (BUS_ID == eUART_1_BUS_ID || BUS_ID == eUART_6_BUS_ID) {       \
            PeriphClkInitStruct.Usart234578ClockSelection = 0;            \
            PeriphClkInitStruct.Usart16ClockSelection     = CLKSELECTION; \
        }                                                                 \
        if (HAL_RCCEx_PeriphCLKConfig (&PeriphClkInitStruct) != HAL_OK) { \
            *(pSTATUS) = eSTATUS_FAILURE;                                 \
            return;                                                       \
        }                                                                 \
        *(pSTATUS) = eSTATUS_SUCCESS;                                     \
    } while (0)

static eSTATUS_t UARTClockInit (eBUS_ID_t busId) {

    eSTATUS_t status = eSTATUS_SUCCESS;
    if (busId == eUART_1_BUS_ID) {
        __HAL_RCC_USART1_CLK_ENABLE ();
        UART_CLOCK_INIT (&status, busId, RCC_PERIPHCLK_USART1, RCC_USART16CLKSOURCE_D2PCLK2);
        return status;
    }
    if (busId == eUART_2_BUS_ID) {
        __HAL_RCC_USART2_CLK_ENABLE ();
        UART_CLOCK_INIT (&status, busId, RCC_PERIPHCLK_USART2, RCC_USART234578CLKSOURCE_D2PCLK1);
        return status;
    }
    if (busId == eUART_3_BUS_ID) {
        __HAL_RCC_USART3_CLK_ENABLE ();
        UART_CLOCK_INIT (&status, busId, RCC_PERIPHCLK_USART3, RCC_USART234578CLKSOURCE_D2PCLK1);
        return status;
    }
    return eSTATUS_FAILURE;
}

static eSTATUS_t
UARTInitGPIO (vUARTBus_t* pBus, UARTInitConf_t conf, UARTBoardConf_t boardConf) {

    eSTATUS_t status       = eSTATUS_SUCCESS;
    eBUS_ID_t busId        = pBus->busId;
    eGPIO_ID_t txId        = boardConf.txId;
    eGPIO_ID_t rxId        = boardConf.rxId;
    uint16_t gpioAlternate = boardConf.gpioAlternate;
    GPIO_INIT_UART (&status, busId, txId, rxId, gpioAlternate);
    return status;
}

eSTATUS_t UARTInit (UARTInitConf_t conf, UARTBoardConf_t boardConf) {

    eBUS_ID_t busId       = boardConf.header.busId;
    eDEVICE_ID_t deviceId = conf.deviceId;
    uint32_t baudRate     = boardConf.baudRate;

    if (BUS_ID_IS_UART (busId) == FALSE || baudRate == 0) {
        return eSTATUS_FAILURE;
    }

    vUARTBus_t* pBus = UARTGetBusById (busId);
    if (pBus->isInitialized == TRUE) {
        return eSTATUS_FAILURE;
    }

    memset (pBus, 0, sizeof (vUARTBus_t));
    pBus->busId                              = busId;
    pBus->deviceId                           = deviceId;
    pBus->handle.Instance                    = UARTGetInstanceById (busId);
    pBus->handle.Init.BaudRate               = baudRate;
    pBus->handle.Init.WordLength             = UART_WORDLENGTH_8B;
    pBus->handle.Init.StopBits               = UART_STOPBITS_1;
    pBus->handle.Init.Parity                 = UART_PARITY_NONE;
    pBus->handle.Init.Mode                   = UART_MODE_TX_RX;
    pBus->handle.Init.HwFlowCtl              = UART_HWCONTROL_NONE;
    pBus->handle.Init.OverSampling           = UART_OVERSAMPLING_16;
    pBus->handle.Init.OneBitSampling         = UART_ONE_BIT_SAMPLE_DISABLE;
    pBus->handle.Init.ClockPrescaler         = UART_PRESCALER_DIV1;
    pBus->handle.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

    if (UARTClockInit (pBus->busId) != eSTATUS_SUCCESS) {
        goto error;
    }

    if (HAL_UART_Init (&pBus->handle) != HAL_OK) {
        goto error;
    }

    if (UARTInitGPIO (pBus, conf, boardConf) != eSTATUS_SUCCESS) {
        goto error;
    }

    if (HAL_UARTEx_SetTxFifoThreshold (&pBus->handle, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK) {
        goto error;
    }

    if (HAL_UARTEx_SetRxFifoThreshold (&pBus->handle, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK) {
        goto error;
    }

    if (HAL_UARTEx_DisableFifoMode (&pBus->handle) != HAL_OK) {
        goto error;
    }

    pBus->isInitialized = TRUE;
    return eSTATUS_SUCCESS;
error:
    memset (pBus, 0, sizeof (UARTBus_t));
    return eSTATUS_FAILURE;
}

eSTATUS_t UARTRead_Blocking (eBUS_ID_t busId, uint8_t* pData, uint32_t size) {

    if (pData == NULL || size == 0) {
        return eSTATUS_FAILURE;
    }

    vUARTBus_t* pBus = UARTGetBusById (busId);
    if (pBus == NULL) {
        return eSTATUS_FAILURE;
    }

    if (HAL_UART_Receive (&pBus->handle, pData, size, HAL_MAX_DELAY) != HAL_OK) {
        return eSTATUS_FAILURE;
    }

    return eSTATUS_SUCCESS;
}

eSTATUS_t UARTWrite_Blocking (eBUS_ID_t busId, uint8_t const* pData, uint32_t size) {

    if (pData == NULL || size == 0) {
        return eSTATUS_FAILURE;
    }

    vUARTBus_t* pBus = UARTGetBusById (busId);
    if (pBus == NULL) {
        return eSTATUS_FAILURE;
    }

    if (HAL_UART_Transmit (&pBus->handle, (uint8_t*)pData, size, HAL_MAX_DELAY) != HAL_OK) {
        return eSTATUS_FAILURE;
    }

    return eSTATUS_SUCCESS;
}

eSTATUS_t UARTRead_IT (eBUS_ID_t busId, uint8_t* pData, uint32_t size) {

    if (pData == NULL || size == 0) {
        return eSTATUS_FAILURE;
    }

    vUARTBus_t* pBus = UARTGetBusById (busId);
    if (pBus == NULL) {
        return eSTATUS_FAILURE;
    }

    if (HAL_UART_Receive_IT (&pBus->handle, pData, size) != HAL_OK) {
        return eSTATUS_FAILURE;
    }

    return eSTATUS_SUCCESS;
}

eSTATUS_t UARTRegisterCallback (eBUS_ID_t busId, eUART_CALLBACK_ID_t cbId, UART_Callback_t callback) {

    if (callback == NULL) {
        return eSTATUS_FAILURE;
    }

    vUARTBus_t* pBus = UARTGetBusById (busId);
    if (pBus == NULL) {
        return eSTATUS_FAILURE;
    }

    if (cbId == eUART_CALLBACK_ID_RX) {
        pBus->rxCallback = callback;
    } else if (cbId == eUART_CALLBACK_ID_TX) {
        pBus->txCallback = callback;
    } else if (cbId == eUART_CALLBACK_ID_ERROR) {
        pBus->errorCallback = callback;
    } else {
        return eSTATUS_FAILURE;
    }

    return eSTATUS_SUCCESS;
}

eSTATUS_t UARTEnableInterrupts (eBUS_ID_t busId, uint32_t priority) {

    if (busId == eUART_1_BUS_ID) {
        HAL_NVIC_SetPriority (USART1_IRQn, priority, priority);
        HAL_NVIC_EnableIRQ (USART1_IRQn);
    } else if (busId == eUART_2_BUS_ID) {
        HAL_NVIC_SetPriority (USART2_IRQn, priority, priority);
        HAL_NVIC_EnableIRQ (USART2_IRQn);
    } else if (busId == eUART_3_BUS_ID) {
        HAL_NVIC_SetPriority (USART3_IRQn, priority, priority);
        HAL_NVIC_EnableIRQ (USART3_IRQn);
    } else {
        return eSTATUS_FAILURE;
    }
    return eSTATUS_SUCCESS;
}

BOOL_t UARTIsValid (eBUS_ID_t busId) {
    return IS_BUS_VALID (UARTGetBusById (busId));
}