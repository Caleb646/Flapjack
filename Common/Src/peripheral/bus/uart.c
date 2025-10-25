#include "peripheral/bus/uart.h"
#include "common.h"
#include "hal.h"
#include "mem/mem.h"
#include "peripheral/gpio.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define UART_VALID(pBUS) ((pBUS) != NULL && (pBUS)->isInitialized == true)

// clang-format off
// #define UART_IT_PE                          0x0028U              /*!< UART parity error interruption                 */
// #define UART_IT_TXE                         0x0727U              /*!< UART transmit data register empty interruption */
// #define UART_IT_TXFNF                       0x0727U              /*!< UART TX FIFO not full interruption             */
// #define UART_IT_TC                          0x0626U              /*!< UART transmission complete interruption        */
// #define UART_IT_RXNE                        0x0525U              /*!< UART read data register not empty interruption */
// #define UART_IT_RXFNE                       0x0525U              /*!< UART RXFIFO not empty interruption             */
// #define UART_IT_IDLE                        0x0424U              /*!< UART idle interruption                         */
// #define UART_IT_LBD                         0x0846U              /*!< UART LIN break detection interruption          */
// #define UART_IT_CTS                         0x096AU              /*!< UART CTS interruption                          */
// #define UART_IT_CM                          0x112EU              /*!< UART character match interruption              */
// #define UART_IT_WUF                         0x1476U              /*!< UART wake-up from stop mode interruption       */
// #define UART_IT_RXFF                        0x183FU              /*!< UART RXFIFO full interruption                  */
// #define UART_IT_TXFE                        0x173EU              /*!< UART TXFIFO empty interruption                 */
// #define UART_IT_RXFT                        0x1A7CU              /*!< UART RXFIFO threshold reached interruption     */
// #define UART_IT_TXFT                        0x1B77U              /*!< UART TXFIFO threshold reached interruption     */
// #define UART_IT_RTO                         0x0B3AU              /*!< UART receiver timeout interruption             */
// #define UART_IT_ERR                         0x0060U              /*!< UART error interruption                        */
// #define UART_IT_ORE                         0x0300U              /*!< UART overrun error interruption                */
// #define UART_IT_NE                          0x0200U              /*!< UART noise error interruption                  */
// #define UART_IT_FE                          0x0100U              /*!< UART frame error interruption                  */
// clang-format on
typedef struct {
    uint32_t hwId;
    eBUS_CALLBACK_ID_t cbId;
    eBUS_CALLBACK_SUB_ID_t cbSubId;
} UARTInterruptMapping_t;

static UARTInterruptMapping_t const gUartInterruptMappings[] = {
    { .hwId = UART_IT_TXE, .cbId = eBUS_CALLBACK_ID_TX }, // TXE: Data register empty
    { .hwId = UART_IT_TXFE, .cbId = eBUS_CALLBACK_ID_TX, .cbSubId = eBUS_CALLBACK_SUB_ID_TX_FIFO_EMPTY }, // TXFE: TX FIFO empty
    { .hwId = UART_IT_TC, .cbId = eBUS_CALLBACK_ID_TX, .cbSubId = eBUS_CALLBACK_SUB_ID_TX_COMPLETE }, // TC: Transmission complete

    { .hwId = UART_IT_RXNE, .cbId = eBUS_CALLBACK_ID_RX }, // RXNE: Data register not empty
    { .hwId = UART_IT_RXFF, .cbId = eBUS_CALLBACK_ID_RX, .cbSubId = eBUS_CALLBACK_SUB_ID_RX_FIFO_FULL }, // RXFF: RX FIFO full
    { .hwId = UART_IT_IDLE, .cbId = eBUS_CALLBACK_ID_RX, .cbSubId = eBUS_CALLBACK_SUB_ID_RX_COMPLETE }, // IDLE: Idle line detected

    { .hwId = UART_IT_RTO, .cbId = eBUS_CALLBACK_ID_ERROR }, // UART receiver timeout interruption
    { .hwId = UART_IT_ERR, .cbId = eBUS_CALLBACK_ID_ERROR }, // UART error interruption
    { .hwId = UART_IT_ORE, .cbId = eBUS_CALLBACK_ID_ERROR }, // UART overrun error interruption
    { .hwId = UART_IT_NE, .cbId = eBUS_CALLBACK_ID_ERROR },  // UART noise error interruption
    { .hwId = UART_IT_FE, .cbId = eBUS_CALLBACK_ID_ERROR },  // UART frame error interruption
};

static SHARED_MEM_SECTION UARTBus_t gBuses[eUART_BUS_ID_MAX] = { 0 };

static USART_TypeDef* UARTGetInstanceById (eBUS_ID_t busId) {

    switch (busId) {
    // NOLINTBEGIN(performance-no-int-to-ptr)
    case eUART_1_BUS_ID: return USART1;
    case eUART_2_BUS_ID: return USART2;
    case eUART_3_BUS_ID: return USART3;
    // NOLINTEND(performance-no-int-to-ptr)
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
    BUS_DO_CALLBACK (&pBus->callbacks[eBUS_CALLBACK_ID_TX], eBUS_CALLBACK_SUB_ID_TX_COMPLETE);
}

void HAL_UART_RxCpltCallback (UART_HandleTypeDef* huart) {

    vUARTBus_t* pBus = UARTGetBusByInstance (huart->Instance);
    if (UART_VALID (pBus) == false) {
        return;
    }
    BUS_DO_CALLBACK (&pBus->callbacks[eBUS_CALLBACK_ID_RX], eBUS_CALLBACK_SUB_ID_RX_COMPLETE);
}

void HAL_UART_ErrorCallback (UART_HandleTypeDef* huart) {

    vUARTBus_t* pBus = UARTGetBusByInstance (huart->Instance);
    if (UART_VALID (pBus) == false) {
        return;
    }
    BUS_DO_CALLBACK (&pBus->callbacks[eBUS_CALLBACK_ID_ERROR], 0U);
}

#define UART_CLOCK_INIT(pSTATUS, BUS_ID, PERIPHCLK, CLKSELECTION)         \
    do {                                                                  \
        RCC_PeriphCLKInitTypeDef PeriphClkInitStruct  = { 0 };            \
        PeriphClkInitStruct.PeriphClockSelection      = PERIPHCLK;        \
        PeriphClkInitStruct.Usart234578ClockSelection = CLKSELECTION;     \
        if ((BUS_ID) == eUART_1_BUS_ID || (BUS_ID) == eUART_6_BUS_ID) {   \
            PeriphClkInitStruct.Usart234578ClockSelection = 0;            \
            PeriphClkInitStruct.Usart16ClockSelection     = CLKSELECTION; \
        }                                                                 \
        if (HAL_RCCEx_PeriphCLKConfig (&PeriphClkInitStruct) != HAL_OK) { \
            *(pSTATUS) = eSTATUS_FAILURE;                                 \
        }                                                                 \
    } while (0)

static eSTATUS_t UARTClockInit (eBUS_ID_t busId) {

    eSTATUS_t status = eSTATUS_SUCCESS;
    // NOLINTBEGIN(performance-no-int-to-ptr)
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
    // NOLINTEND(performance-no-int-to-ptr)
    return eSTATUS_FAILURE;
}

static eSTATUS_t UARTInitGPIO (vUARTBus_t* pBus, UARTInitConf_t conf) {

    FJ_UNUSED (pBus);
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

    // if (HAL_UARTEx_DisableFifoMode (&pBus->handle) != HAL_OK) {
    //     goto error;
    // }

    pBus->isInitialized = true;
    return eSTATUS_SUCCESS;
error:
    memset (pBus, 0, sizeof (UARTBus_t));
    return eSTATUS_FAILURE;
}

eSTATUS_t UARTRead_Blocking (vUARTBus_t* pBus, eDEVICE_ID_t deviceId, uint8_t* pData, size_t size) {

    FJ_UNUSED (deviceId);
    if (UART_VALID (pBus) == false || pData == NULL || size == 0) {
        return eSTATUS_FAILURE;
    }

    if (HAL_UART_Receive (&pBus->handle, pData, size, HAL_MAX_DELAY) != HAL_OK) {
        return eSTATUS_FAILURE;
    }

    return eSTATUS_SUCCESS;
}

eSTATUS_t UARTWrite_Blocking (vUARTBus_t* pBus, eDEVICE_ID_t deviceId, uint8_t const* pData, size_t size) {

    FJ_UNUSED (deviceId);
    if (UART_VALID (pBus) == false || pData == NULL || size == 0) {
        return eSTATUS_FAILURE;
    }

    if (HAL_UART_Transmit (&pBus->handle, (uint8_t*)pData, size, HAL_MAX_DELAY) != HAL_OK) {
        return eSTATUS_FAILURE;
    }

    return eSTATUS_SUCCESS;
}

eSTATUS_t UARTRead_IT (vUARTBus_t* pBus, eDEVICE_ID_t deviceId, uint8_t* pData, size_t size) {

    FJ_UNUSED (deviceId);
    if (UART_VALID (pBus) == false || pData == NULL || size == 0) {
        return eSTATUS_FAILURE;
    }

    if (HAL_UART_Receive_IT (&pBus->handle, pData, size) != HAL_OK) {
        return eSTATUS_FAILURE;
    }

    return eSTATUS_SUCCESS;
}

eSTATUS_t UARTRegisterCallback (vUARTBus_t* pBus, eDEVICE_ID_t deviceId, BusCallback_t callback) {

    FJ_UNUSED (deviceId);
    if (UART_VALID (pBus) == false || callback.Callback == NULL ||
        BUS_CALLBACK_ID_VALID (callback.data.cbId) == false) {
        return eSTATUS_FAILURE;
    }

    pBus->callbacks[callback.data.cbId] = callback;
    return UARTEnableInterrupts (pBus, 5);
}

eSTATUS_t UARTEnableInterrupts (vUARTBus_t* pBus, uint32_t priority) {

    if (UART_VALID (pBus) == false) {
        return eSTATUS_FAILURE;
    }

    FOR_EACH_CONST (UARTInterruptMapping_t, gUartInterruptMappings) {

        BusCallback_t* pCb = &pBus->callbacks[pElement->cbId];
        if (pCb->Callback != NULL && BUS_SUB_ID_IS_ACTIVE (pCb->data, pElement->cbSubId)) {
            __HAL_UART_ENABLE_IT (&pBus->handle, pElement->hwId);
        } else {
            __HAL_UART_DISABLE_IT (&pBus->handle, pElement->hwId);
        }
    }

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

eSTATUS_t UART_WRITE_BLOCKING (void* pCtx, eDEVICE_ID_t deviceId, uint8_t const* pData, size_t size) {
    return UARTWrite_Blocking ((vUARTBus_t*)pCtx, deviceId, pData, size);
}

eSTATUS_t UART_READ_IT (void* pCtx, eDEVICE_ID_t deviceId, uint8_t* pData, size_t size) {
    return UARTRead_IT ((vUARTBus_t*)pCtx, deviceId, pData, size);
}

eSTATUS_t UART_REGISTER_CALLBACK (void* pCtx, eDEVICE_ID_t deviceId, BusCallback_t callback) {
    return UARTRegisterCallback ((vUARTBus_t*)pCtx, deviceId, callback);
}