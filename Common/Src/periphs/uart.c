#include "periphs/uart.h"
#include "common.h"
#include "hal.h"
#include "periphs/gpio.h"

#define IS_BUS_VALID(pBus) (pBus != NULL && pBus->isInitialized == TRUE)

static UARTBus_t gBuses[eUART_BUS_ID_MAX] = { 0 };

static UARTBus_t* UARTGetBusById (eUART_BUS_ID_t busId) {

    if (busId >= eUART_BUS_ID_MAX) {
        return NULL;
    }
    return &gBuses[busId];
}

static USART_TypeDef* UARTGetInstanceById (eUART_BUS_ID_t busId) {

    switch (busId) {
    case eUART_1_BUS_ID: return USART1;
    case eUART_2_BUS_ID: return USART2;
    case eUART_3_BUS_ID: return USART3;
    default: return NULL;
    }
}

static UARTBus_t* UARTGetBusByInstance (USART_TypeDef* instance) {

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

    UARTBus_t* pBus = UARTGetBusByInstance (huart->Instance);
    if (IS_BUS_VALID (pBus) == FALSE) {
        return;
    }
    if (pBus->txCallback != NULL) {
        pBus->txCallback (pBus->busId);
    }
}

void HAL_UART_RxCpltCallback (UART_HandleTypeDef* huart) {

    UARTBus_t* pBus = UARTGetBusByInstance (huart->Instance);
    if (IS_BUS_VALID (pBus) == FALSE) {
        return;
    }
    if (pBus->rxCallback != NULL) {
        pBus->rxCallback (pBus->busId);
    }
}

void HAL_UART_ErrorCallback (UART_HandleTypeDef* huart) {

    UARTBus_t* pBus = UARTGetBusByInstance (huart->Instance);
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

static eSTATUS_t UARTClockInit (eUART_BUS_ID_t busId) {

    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = { 0 };
    if (busId == eUART_1_BUS_ID) {

        __HAL_RCC_USART1_CLK_ENABLE ();
        PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART1;
        PeriphClkInitStruct.Usart16ClockSelection = RCC_USART16CLKSOURCE_D2PCLK2;

        if (HAL_RCCEx_PeriphCLKConfig (&PeriphClkInitStruct) != HAL_OK) {
            return eSTATUS_FAILURE;
        }
    } else if (busId == eUART_2_BUS_ID) {

        __HAL_RCC_USART2_CLK_ENABLE ();
        PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART2;
        PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;

        if (HAL_RCCEx_PeriphCLKConfig (&PeriphClkInitStruct) != HAL_OK) {
            return eSTATUS_FAILURE;
        }
    } else if (busId == eUART_3_BUS_ID) {

        __HAL_RCC_USART3_CLK_ENABLE ();
        PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3;
        PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;

        if (HAL_RCCEx_PeriphCLKConfig (&PeriphClkInitStruct) != HAL_OK) {
            return eSTATUS_FAILURE;
        }
    } else {
        return eSTATUS_FAILURE;
    }
    return eSTATUS_SUCCESS;
}

eSTATUS_t UARTInit (UARTInitConf_t const* pConf) {

    if (pConf == NULL) {
        return eSTATUS_FAILURE;
    }

    UARTBus_t* pBus = UARTGetBusById (pConf->busId);
    if (pBus == NULL) {
        return eSTATUS_FAILURE;
    }

    if (pBus->isInitialized == TRUE) {
        return eSTATUS_FAILURE;
    }

    memset (pBus, 0, sizeof (UARTBus_t));
    pBus->busId                      = pConf->busId;
    pBus->baudRate                   = pConf->baudRate;
    pBus->deviceId                   = pConf->deviceId;
    pBus->isInitialized              = TRUE;
    pBus->handle.Instance            = UARTGetInstanceById (pConf->busId);
    pBus->handle.Init.BaudRate       = pConf->baudRate;
    pBus->handle.Init.WordLength     = UART_WORDLENGTH_8B;
    pBus->handle.Init.StopBits       = UART_STOPBITS_1;
    pBus->handle.Init.Parity         = UART_PARITY_NONE;
    pBus->handle.Init.Mode           = UART_MODE_TX_RX;
    pBus->handle.Init.HwFlowCtl      = UART_HWCONTROL_NONE;
    pBus->handle.Init.OverSampling   = UART_OVERSAMPLING_16;
    pBus->handle.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    pBus->handle.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    pBus->handle.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

    if (UARTClockInit (pBus->busId) != eSTATUS_SUCCESS) {
        goto error;
    }

    if (HAL_UART_Init (&pBus->handle) != HAL_OK) {
        goto error;
    }

    if (GPIOInitUART (pBus->handle.Instance, &pBus->gpio) != eSTATUS_SUCCESS) {
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

    return eSTATUS_SUCCESS;
error:
    memset (pBus, 0, sizeof (UARTBus_t));
    return eSTATUS_FAILURE;
}

eSTATUS_t UARTRead_Blocking (eUART_BUS_ID_t busId, uint8_t* pData, uint16_t size) {

    if (pData == NULL || size == 0) {
        return eSTATUS_FAILURE;
    }

    UARTBus_t* pBus = UARTGetBusById (busId);
    if (IS_BUS_VALID (pBus) == FALSE) {
        return eSTATUS_FAILURE;
    }

    if (HAL_UART_Receive (&pBus->handle, pData, size, HAL_MAX_DELAY) != HAL_OK) {
        return eSTATUS_FAILURE;
    }

    return eSTATUS_SUCCESS;
}

eSTATUS_t UARTWrite_Blocking (eUART_BUS_ID_t busId, uint8_t const* pData, uint16_t size) {

    if (pData == NULL || size == 0) {
        return eSTATUS_FAILURE;
    }

    UARTBus_t* pBus = UARTGetBusById (busId);
    if (IS_BUS_VALID (pBus) == FALSE) {
        return eSTATUS_FAILURE;
    }

    if (HAL_UART_Transmit (&pBus->handle, (uint8_t*)pData, size, HAL_MAX_DELAY) != HAL_OK) {
        return eSTATUS_FAILURE;
    }

    return eSTATUS_SUCCESS;
}

eSTATUS_t UARTRead_IT (eUART_BUS_ID_t busId, uint8_t* pData, uint16_t size) {

    if (pData == NULL || size == 0) {
        return eSTATUS_FAILURE;
    }

    UARTBus_t* pBus = UARTGetBusById (busId);
    if (IS_BUS_VALID (pBus) == FALSE) {
        return eSTATUS_FAILURE;
    }

    if (HAL_UART_Receive_IT (&pBus->handle, pData, size) != HAL_OK) {
        return eSTATUS_FAILURE;
    }

    return eSTATUS_SUCCESS;
}

eSTATUS_t UARTRegisterCallback (eUART_BUS_ID_t busId, eUART_CALLBACK_ID_t cbId, UART_Callback_t callback) {

    if (callback == NULL) {
        return eSTATUS_FAILURE;
    }

    UARTBus_t* pBus = UARTGetBusById (busId);
    if (IS_BUS_VALID (pBus) == FALSE) {
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

eSTATUS_t UARTEnableInterrupt (eUART_BUS_ID_t busId, uint32_t priority) {

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

BOOL_t UARTIsValid (eUART_BUS_ID_t busId) {
    return IS_BUS_VALID (UARTGetBusById (busId));
}