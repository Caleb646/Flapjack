#include "peripheral/bus/uart.h"
#include "common.h"
#include "hal.h"
#include "mem/mem.h"
#include "peripheral/gpio.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define UART_VALID(pBUS) ((pBUS) != NULL && (pBUS)->isInitialized == true)

static SHARED_MEM_SECTION UARTBus_t gBuses[eUART_BUS_ID_MAX] = { 0 };

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
    if (UART_VALID (pBus) == false) {
        return;
    }
    if (pBus->txCallback != NULL) {
        pBus->txCallback (pBus->busId);
    }
}

void HAL_UART_RxCpltCallback (UART_HandleTypeDef* huart) {

    vUARTBus_t* pBus = UARTGetBusByInstance (huart->Instance);
    if (UART_VALID (pBus) == false) {
        return;
    }
    if (pBus->rxCallback != NULL) {
        pBus->rxCallback (pBus->busId);
    }
}

void HAL_UART_ErrorCallback (UART_HandleTypeDef* huart) {

    vUARTBus_t* pBus = UARTGetBusByInstance (huart->Instance);
    if (UART_VALID (pBus) == false) {
        return;
    }
    if (pBus->errorCallback != NULL) {
        pBus->errorCallback (pBus->busId);
    }
}

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
        }                                                                 \
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

static eSTATUS_t UARTInitGPIO (vUARTBus_t* pBus, UARTInitConf_t conf) {

    eSTATUS_t status = eSTATUS_SUCCESS;

    eDEVICE_ID_t deviceId               = conf.deviceBoardConf.deviceId;
    UARTBoardConf_t boardConf           = conf.busBoardConf.UARTBoardConf;
    GPIOBoardConf_t const* pTxBoardConf = boardConf.pTxBoardConf;
    GPIOBoardConf_t const* pRxBoardConf = boardConf.pRxBoardConf;

    if (pTxBoardConf == NULL || pRxBoardConf == NULL) {
        return eSTATUS_FAILURE;
    }

    GPIO_INIT (&status, deviceId, *pTxBoardConf);
    if (STATUS_FAIL (status)) {
        return status;
    }
    GPIO_INIT (&status, deviceId, *pRxBoardConf);
    if (STATUS_FAIL (status)) {
        return status;
    }
    return status;
}

vUARTBus_t* UARTGetBusById (eBUS_ID_t busId) {

    uint32_t busIndex = UART_BUS_ID2IDX (busId);
    if (BUS_ID_IS_UART (busId) == false || busIndex >= eUART_BUS_ID_MAX) {
        return NULL;
    }
    return &gBuses[busIndex];
}

eSTATUS_t UARTInit (UARTInitConf_t conf, vUARTBus_t* pOutBus) {

    DeviceBoardConf_t device = conf.deviceBoardConf;
    eDEVICE_ID_t deviceId    = device.deviceId;

    BusBoardConf_t bus = conf.busBoardConf;
    eBUS_ID_t busId    = bus.busId;
    if (BUS_ID_IS_UART (busId) == false) {
        return eSTATUS_FAILURE;
    }

    UARTBoardConf_t uart = bus.UARTBoardConf;
    uint32_t baudRate    = uart.baudRate;
    if (baudRate == 0) {
        return eSTATUS_FAILURE;
    }

    vUARTBus_t* pBus = UARTGetBusById (busId);
    if (pOutBus != NULL) {
        pBus = pOutBus;
    }
    if (pBus->isInitialized == true) {
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

    if (UARTInitGPIO (pBus, conf) != eSTATUS_SUCCESS) {
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

    pBus->isInitialized = true;
    return eSTATUS_SUCCESS;
error:
    memset (pBus, 0, sizeof (UARTBus_t));
    return eSTATUS_FAILURE;
}

eSTATUS_t
UARTRead_Blocking (vUARTBus_t* pBus, eDEVICE_ID_t deviceId, uint8_t* pData, size_t size) {

    if (UART_VALID (pBus) == false || pData == NULL || size == 0) {
        return eSTATUS_FAILURE;
    }

    if (HAL_UART_Receive (&pBus->handle, pData, size, HAL_MAX_DELAY) != HAL_OK) {
        return eSTATUS_FAILURE;
    }

    return eSTATUS_SUCCESS;
}

eSTATUS_t
UARTWrite_Blocking (vUARTBus_t* pBus, eDEVICE_ID_t deviceId, uint8_t const* pData, size_t size) {

    if (UART_VALID (pBus) == false || pData == NULL || size == 0) {
        return eSTATUS_FAILURE;
    }

    if (HAL_UART_Transmit (&pBus->handle, (uint8_t*)pData, size, HAL_MAX_DELAY) != HAL_OK) {
        return eSTATUS_FAILURE;
    }

    return eSTATUS_SUCCESS;
}

eSTATUS_t UARTRead_IT (vUARTBus_t* pBus, eDEVICE_ID_t deviceId, uint8_t* pData, size_t size) {

    if (UART_VALID (pBus) == false || pData == NULL || size == 0) {
        return eSTATUS_FAILURE;
    }

    if (HAL_UART_Receive_IT (&pBus->handle, pData, size) != HAL_OK) {
        return eSTATUS_FAILURE;
    }

    return eSTATUS_SUCCESS;
}

eSTATUS_t UARTRegisterCallback (vUARTBus_t* pBus, eUART_CALLBACK_ID_t cbId, UART_Callback_t callback) {

    if (UART_VALID (pBus) == false || callback == NULL) {
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

eSTATUS_t UARTEnableInterrupts (vUARTBus_t* pBus, uint32_t priority) {

    if (UART_VALID (pBus) == false) {
        return eSTATUS_FAILURE;
    }

    // UART read data register not empty interrupt
    __HAL_UART_ENABLE_IT (&pBus->handle, UART_IT_RXNE);
    // UART error interrupt
    __HAL_UART_ENABLE_IT (&pBus->handle, UART_IT_ERR);

    switch (pBus->busId) {
    case eUART_1_BUS_ID:
        HAL_NVIC_SetPriority (USART1_IRQn, priority, priority);
        HAL_NVIC_EnableIRQ (USART1_IRQn);
        break;
    case eUART_2_BUS_ID:
        HAL_NVIC_SetPriority (USART2_IRQn, priority, priority);
        HAL_NVIC_EnableIRQ (USART2_IRQn);
        break;
    case eUART_3_BUS_ID:
        HAL_NVIC_SetPriority (USART3_IRQn, priority, priority);
        HAL_NVIC_EnableIRQ (USART3_IRQn);
        break;
    case eUART_4_BUS_ID:
        HAL_NVIC_SetPriority (UART4_IRQn, priority, priority);
        HAL_NVIC_EnableIRQ (UART4_IRQn);
        break;
    case eUART_5_BUS_ID:
        HAL_NVIC_SetPriority (UART5_IRQn, priority, priority);
        HAL_NVIC_EnableIRQ (UART5_IRQn);
        break;
    default: return eSTATUS_FAILURE;
    }
    return eSTATUS_SUCCESS;
}

eSTATUS_t UART_READ_BLOCKING (void* pCtx, eDEVICE_ID_t deviceId, uint8_t* pData, size_t size) {
    return UARTRead_Blocking ((vUARTBus_t*)pCtx, deviceId, pData, size);
}

eSTATUS_t
UART_WRITE_BLOCKING (void* pCtx, eDEVICE_ID_t deviceId, uint8_t const* pData, size_t size) {
    return UARTWrite_Blocking ((vUARTBus_t*)pCtx, deviceId, pData, size);
}

eSTATUS_t UART_READ_IT (void* pCtx, eDEVICE_ID_t deviceId, uint8_t* pData, size_t size) {
    return UARTRead_IT ((vUARTBus_t*)pCtx, deviceId, pData, size);
}