#include <stdbool.h>
#include <stdint.h>

#include "common.h"

#include "drivers/serial/serial.h"

#include "drivers/io/gpio.h"
#include "drivers/io/gpio_defs.h"

#include "drivers/dma.h"
#include "drivers/dma_defs.h"

#include "platform/platform.h"

#include "targets/target.h"

FJ_TESTABLE eSTATUS_t Stm32_Uart_Write (SerialPort_t* pSerialPort, uint8_t const* pData, uint32_t nData);
FJ_TESTABLE eSTATUS_t Stm32_Uart_Read (SerialPort_t* pSerialPort, uint8_t* pData, uint32_t nData);
FJ_TESTABLE eSTATUS_t Stm32_Uart_SetBaud (SerialPort_t* pSerialPort, eSERIAL_PORT_BAUD_t baudrate);
static UartDevice_t* Stm32_Uart_GetByPortId (eSERIAL_PORT_ID_t portId);

FJ_DEFINE_SHARED (SerialPortVtable_t, e_UartVtbl) = {
    .fnWrite   = Stm32_Uart_Write,
    .fnRead    = Stm32_Uart_Read,
    .fnSetBaud = Stm32_Uart_SetBaud,
};

FJ_DEFINE_SHARED (UartDevice_t, e_UartDevices[]) = {
#if TARG_UART_ENABLED(1)
    {
    .portId = eSERIAL_PORT_ID_UART1,
    },
#endif
#if TARG_UART_ENABLED(2)
    {
    .portId = eSERIAL_PORT_ID_UART2,
    },
#endif
#if TARG_UART_ENABLED(3)
    {
    .portId = eSERIAL_PORT_ID_UART3,
    },
#endif
#if TARG_UART_ENABLED(4)
    {
    .portId = eSERIAL_PORT_ID_UART4,
    },
#endif
#if TARG_UART_ENABLED(5)
    {
    .portId = eSERIAL_PORT_ID_UART5,
    },
#endif
};

FJ_DEFINE_SHARED (UartHwCfg_t, e_UartHwCfgs[]) = {
#if TARG_UART_ENABLED(1)
    {
    .portId          = eSERIAL_PORT_ID_UART1,
    .pInstance       = USART1,
    .pRccReg         = &RCC->APB2ENR,
    .rccMask         = RCC_APB2ENR_USART1EN,
    .txOpts          = { PLAT_GPIO_ID_MAKE (A, 9) },
    .rxOpts          = { PLAT_GPIO_ID_MAKE (A, 10) },
    .afOpts          = { GPIO_AF7_USART1 },
    .irqNum          = USART1_IRQn,
    .dma_RxRequestId = DMA_REQUEST_USART1_RX,
    .dma_TxRequestId = DMA_REQUEST_USART1_TX,
    },
#endif
#if TARG_UART_ENABLED(2)
    {
    .portId          = eSERIAL_PORT_ID_UART2,
    .pInstance       = USART2,
    .pRccReg         = &RCC->APB1LENR,
    .rccMask         = RCC_APB1LENR_USART2EN,
    .txOpts          = { PLAT_GPIO_ID_MAKE (A, 2) },
    .rxOpts          = { PLAT_GPIO_ID_MAKE (A, 3) },
    .afOpts          = { GPIO_AF7_USART2 },
    .irqNum          = USART2_IRQn,
    .dma_RxRequestId = DMA_REQUEST_USART2_RX,
    .dma_TxRequestId = DMA_REQUEST_USART2_TX,
    },
#endif
};

static void Stm32_Uart_DmaIrqHandler (DmaDevice_t* pDmaDevice, void* pCtx) {

    // SerialPort_t* pSerialPort = (SerialPort_t*)pCtx;
    // if (!pSerialPort) {
    //     return;
    // }

    // UartDevice_t* pUartDev = (UartDevice_t*)pSerialPort->pSerialHwHandle;
    // if (!pUartDev) {
    //     return;
    // }

    // HAL_UART_DMA_IRQHandler (&pUartDev->handle);
    // HAL_DMA_IRQHandler (&pDmaDevice->handle.Parent);
    HAL_DMA_IRQHandler (&pDmaDevice->handle);
}

void Stm32_Uart_IrqHandler (eSERIAL_PORT_ID_t portId) {

    UartDevice_t* pUartDev    = Stm32_Uart_GetByPortId (portId);
    SerialPort_t* pSerialPort = pUartDev ? pUartDev->pSerialPort : NULL;
    if (!pUartDev || !pSerialPort) {
        return;
    }

    UART_HandleTypeDef* huart = &pUartDev->handle;
    uint32_t isrflags         = READ_REG (huart->Instance->ISR);
    uint32_t cr1its           = READ_REG (huart->Instance->CR1);
    uint32_t cr3its           = READ_REG (huart->Instance->CR3);

    /* UART parity error interrupt occurred -------------------------------------*/
    if (((isrflags & USART_ISR_PE) != 0U) && ((cr1its & USART_CR1_PEIE) != 0U)) {
        __HAL_UART_CLEAR_FLAG (huart, UART_CLEAR_PEF);
    }

    /* UART frame error interrupt occurred --------------------------------------*/
    if (((isrflags & USART_ISR_FE) != 0U) && ((cr3its & USART_CR3_EIE) != 0U)) {
        __HAL_UART_CLEAR_FLAG (huart, UART_CLEAR_FEF);
    }

    /* UART noise error interrupt occurred --------------------------------------*/
    if (((isrflags & USART_ISR_NE) != 0U) && ((cr3its & USART_CR3_EIE) != 0U)) {
        __HAL_UART_CLEAR_FLAG (huart, UART_CLEAR_NEF);
    }

    /* UART Over-Run interrupt occurred -----------------------------------------*/
    if (((isrflags & USART_ISR_ORE) != 0U) &&
        (((cr1its & USART_CR1_RXNEIE_RXFNEIE) != 0U) || ((cr3its & (USART_CR3_RXFTIE | USART_CR3_EIE)) != 0U))) {
        __HAL_UART_CLEAR_FLAG (huart, UART_CLEAR_OREF);
    }

    /* UART Receiver Timeout interrupt occurred ---------------------------------*/
    if (((isrflags & USART_ISR_RTOF) != 0U) && ((cr1its & USART_CR1_RTOIE) != 0U)) {
        __HAL_UART_CLEAR_FLAG (huart, UART_CLEAR_RTOF);
    }

    /* UART in mode Receiver ---------------------------------------------------*/
    if (((isrflags & USART_ISR_RXNE_RXFNE) != 0U) &&
        (((cr1its & USART_CR1_RXNEIE_RXFNEIE) != 0U) || ((cr3its & USART_CR3_RXFTIE) != 0U))) {
        // TODO
        return;
    }

    /* UART Transmission Complete interrupt occurred ---------------------------------*/
    if (((isrflags & USART_ISR_TC) != 0U) && ((cr1its & USART_CR1_TCIE) != 0U)) {
        __HAL_UART_CLEAR_IT (huart, UART_CLEAR_TCF);

        if (pSerialPort->pTxDmaDev && RingBuffIsValid (&pSerialPort->txRingBuff)) {

            RingBuff* pTxRingBuff = &pSerialPort->txRingBuff;
            RingBuffSkip (pTxRingBuff, pSerialPort->txBytesInProgress);

            if (RingBuffGetFull (pTxRingBuff) > 0) {

                void* pSendStart               = RingBuffGetLinearBlockReadAddress (pTxRingBuff);
                uint32_t sendSize              = RingBuffGetLinearBlockReadSize (pTxRingBuff);
                pSerialPort->txBytesInProgress = sendSize;
                pSerialPort->isTxBusy          = true;
                HAL_UART_Transmit_DMA (&pUartDev->handle, (uint8_t*)pSendStart, sendSize);

            } else {
                pSerialPort->txBytesInProgress = 0;
                pSerialPort->isTxBusy          = false;
            }
        }
    }

    // HAL_UART_IRQHandler (&pUartDev->handle);
}

#define UART_IRQ_HANDLER_DEF(TYPE, NUM, PORT_ID)               \
    void TYPE##NUM##_IRQHandler (void) {                       \
        Stm32_Uart_IrqHandler (eSERIAL_PORT_ID_UART##PORT_ID); \
    }

#if TARG_UART_ENABLED(1)
UART_IRQ_HANDLER_DEF (USART, 1, 1);
#endif

#if TARG_UART_ENABLED(2)
UART_IRQ_HANDLER_DEF (USART, 2, 2);
#endif

#if TARG_UART_ENABLED(3)
UART_IRQ_HANDLER_DEF (USART, 3, 3);
#endif

#if TARG_UART_ENABLED(4)
UART_IRQ_HANDLER_DEF (UART, 4, 4);
#endif

#if TARG_UART_ENABLED(5)
UART_IRQ_HANDLER_DEF (UART, 5, 5);
#endif

static UartDevice_t* Stm32_Uart_GetByPortId (eSERIAL_PORT_ID_t portId) {

    for (uint32_t i = 0; i < sizeof (e_UartDevices) / sizeof (UartDevice_t); ++i) {
        if (e_UartDevices[i].portId == portId) {
            return &e_UartDevices[i];
        }
    }
    return NULL;
}

static UartHwCfg_t* Stm32_Uart_GetHwCfgByPortId (eSERIAL_PORT_ID_t portId) {

    for (uint32_t i = 0; i < sizeof (e_UartHwCfgs) / sizeof (UartHwCfg_t); ++i) {
        if (e_UartHwCfgs[i].portId == portId) {
            return &e_UartHwCfgs[i];
        }
    }
    return NULL;
}

FJ_TESTABLE eSTATUS_t
Stm32_Uart_SetupDma (SerialPortCfg_t const* pCfg, UartHwCfg_t const* pHwCfg, UartDevice_t* pUartDev, bool isRx, SerialPort_t* pOutSerialPort) {

    DmaCfg_t dmaCfg = { 0 };
    if (isRx) {
        dmaCfg.direction = eDMA_DIRECTION_PERIPH_TO_MEM;
        dmaCfg.mode      = eDMA_MODE_CIRCULAR;
        dmaCfg.requestId = pHwCfg->dma_RxRequestId;
    } else {
        dmaCfg.direction = eDMA_DIRECTION_MEM_TO_PERIPH;
        dmaCfg.mode      = eDMA_MODE_NORMAL;
        dmaCfg.requestId = pHwCfg->dma_TxRequestId;
    }

    dmaCfg.fnIrqHandler = Stm32_Uart_DmaIrqHandler;
    dmaCfg.pCtx         = (void*)pOutSerialPort;

    DmaDevice_t** pDmaDev = isRx ? &pOutSerialPort->pRxDmaDev : &pOutSerialPort->pTxDmaDev;
    *pDmaDev              = Dma_Init (&dmaCfg);
    if (!(*pDmaDev)) {
        return eSTATUS_FAIL;
    }

    if (isRx) {
        pOutSerialPort->isRxBusy = false;
    } else {
        pOutSerialPort->isTxBusy = false;
    }

    uint32_t bufferSize = isRx ? pCfg->rxBufferSize : pCfg->txBufferSize;
    uint8_t* pBufData   = Alloc_SharedMem (bufferSize);
    if (!pBufData) {
        return eSTATUS_FAIL;
    }

    // Initialize ring buffer
    RingBuff* pRingBuff = isRx ? &pOutSerialPort->rxRingBuff : &pOutSerialPort->txRingBuff;
    RingBuffInit (pBufData, bufferSize, pRingBuff);

    // Link DMA to UART handle
    if (isRx) {
        __HAL_LINKDMA (&(pUartDev->handle), hdmarx, (*pDmaDev)->handle);
        // Start RX DMA transfer
        if (HAL_UART_Receive_DMA (&pUartDev->handle, RingBuffGetBufferData (pRingBuff), bufferSize) != HAL_OK) {
            return eSTATUS_FAIL;
        }
    } else {
        __HAL_LINKDMA (&(pUartDev->handle), hdmatx, (*pDmaDev)->handle);
    }

    return eSTATUS_OK;
}

FJ_TESTABLE eSTATUS_t Stm32_Uart_Write (SerialPort_t* pSerialPort, uint8_t const* pData, uint32_t nData) {

    UartDevice_t* pUartDev = (UartDevice_t*)pSerialPort->pSerialHwHandle;
    if (pSerialPort->pTxDmaDev && RingBuffIsValid (&pSerialPort->txRingBuff)) {

        RingBuff* pTxRingBuff = &pSerialPort->txRingBuff;
        RingBuffWrite (pTxRingBuff, (uint8_t*)pData, nData);

        // dma is already in progress. once the current transfer is complete,
        // the irq handler will check for more data in the ring buffer
        if (pSerialPort->isTxBusy || pSerialPort->txBytesInProgress > 0) {
            return eSTATUS_OK;
        }

        // start new dma transfer and only send a linear block.
        // the irq handler will send the bytes that wrap around.
        void* pSendStart               = RingBuffGetLinearBlockReadAddress (pTxRingBuff);
        uint32_t sendSize              = RingBuffGetLinearBlockReadSize (pTxRingBuff);
        pSerialPort->txBytesInProgress = sendSize;
        pSerialPort->isTxBusy          = true;
        if (HAL_UART_Transmit_DMA (&pUartDev->handle, (uint8_t*)pSendStart, sendSize) != HAL_OK) {
            return eSTATUS_FAIL;
        }

    } else {

        if (HAL_UART_Transmit (&pUartDev->handle, (uint8_t*)pData, nData, HAL_MAX_DELAY) != HAL_OK) {
            return eSTATUS_FAIL;
        }
    }
    return eSTATUS_OK;
}

// Return the total number of bytes read
FJ_TESTABLE uint32_t Stm32_Uart_Read (SerialPort_t* pSerialPort, uint8_t* pData, uint32_t nData) {

    UartDevice_t* pUartDev = (UartDevice_t*)pSerialPort->pSerialHwHandle;

    if (pSerialPort->pRxDmaDev && RingBuffIsValid (&pSerialPort->rxRingBuff)) {

        uint32_t bytesRemaining     = __HAL_DMA_GET_COUNTER (&pSerialPort->pRxDmaDev->handle);
        uint32_t totalBytesInBuff   = RingBuffGetSize (&pSerialPort->rxRingBuff);
        uint32_t totalBytesReceived = totalBytesInBuff - bytesRemaining;
        uint32_t totalBytesWritten  = RingBuffGetFree (&pSerialPort->rxRingBuff);
        if (totalBytesReceived > totalBytesWritten) {
            RingBuffAdvance (&pSerialPort->rxRingBuff, totalBytesReceived - totalBytesWritten);
        }
        return RingBuffRead (&pSerialPort->rxRingBuff, pData, nData);
    }

    if (HAL_UART_Receive (&pUartDev->handle, pData, nData, HAL_MAX_DELAY) != HAL_OK) {
        return 0;
    }
    return nData;
}

FJ_TESTABLE eSTATUS_t Stm32_Uart_SetBaud (SerialPort_t* pSerialPort, eSERIAL_PORT_BAUD_t baudrate) {

    // TODO
    return eSTATUS_FAIL;
}

eSTATUS_t Plat_Uart_Init (SerialPortCfg_t const* pCfg, SerialPort_t* pOutSerialPort) {

    if (!pCfg || !pOutSerialPort) {
        return eSTATUS_FAIL;
    }

    UartDevice_t* pUartDev = Stm32_Uart_GetByPortId (pCfg->portId);
    if (!pUartDev) {
        return eSTATUS_FAIL;
    }

    if (pUartDev->pSerialPort) {
        return eSTATUS_OK; // already initialized
    }

    UartHwCfg_t* pHwCfg = Stm32_Uart_GetHwCfgByPortId (pCfg->portId);
    if (!pHwCfg || !pHwCfg->pInstance) {
        return eSTATUS_FAIL;
    }

    eGPIO_ID_t txPin = 0U;
    eGPIO_ID_t rxPin = 0U;
    switch (pCfg->portId) {
    case eSERIAL_PORT_ID_UART1:
        txPin = TARG_UART_1_TX;
        rxPin = TARG_UART_1_RX;
        break;
    case eSERIAL_PORT_ID_UART2:
        txPin = TARG_UART_2_TX;
        rxPin = TARG_UART_2_RX;
        break;
    case eSERIAL_PORT_ID_UART3:
        txPin = TARG_UART_3_TX;
        rxPin = TARG_UART_3_RX;
        break;
    case eSERIAL_PORT_ID_UART4:
        txPin = TARG_UART_4_TX;
        rxPin = TARG_UART_4_RX;
        break;
    case eSERIAL_PORT_ID_UART5:
        txPin = TARG_UART_5_TX;
        rxPin = TARG_UART_5_RX;
        break;
    default: return eSTATUS_FAIL;
    }

    bool found = false;
    for (size_t i = 0U; i < PLAT_UART_MAX_PIN_SEL; ++i) {
        if (pHwCfg->txOpts[i] == txPin && pHwCfg->rxOpts[i] == rxPin) {
            GPIO_Init (txPin, pCfg->portId, PLAT_GPIO_CFG_UART, pHwCfg->afOpts[i]);
            GPIO_Init (rxPin, pCfg->portId, PLAT_GPIO_CFG_UART, pHwCfg->afOpts[i]);
            found = true;
            break;
        }
    }

    if (!found) {
        return eSTATUS_FAIL;
    }

    *(pHwCfg->pRccReg) |= pHwCfg->rccMask; // enable rcc
    pUartDev->pSerialPort                        = pOutSerialPort;
    pUartDev->handle.Instance                    = pHwCfg->pInstance;
    pUartDev->handle.Init.BaudRate               = pCfg->baudrate;
    pUartDev->handle.Init.WordLength             = UART_WORDLENGTH_8B;
    pUartDev->handle.Init.StopBits               = UART_STOPBITS_1;
    pUartDev->handle.Init.Parity                 = UART_PARITY_NONE;
    pUartDev->handle.Init.Mode                   = UART_MODE_TX_RX;
    pUartDev->handle.Init.HwFlowCtl              = UART_HWCONTROL_NONE;
    pUartDev->handle.Init.OverSampling           = UART_OVERSAMPLING_16;
    pUartDev->handle.Init.OneBitSampling         = UART_ONE_BIT_SAMPLE_DISABLE;
    pUartDev->handle.Init.ClockPrescaler         = UART_PRESCALER_DIV1;
    pUartDev->handle.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

    if (HAL_UART_DeInit (&pUartDev->handle) != HAL_OK) {
        return eSTATUS_FAIL;
    }

    if (HAL_UART_Init (&pUartDev->handle) != HAL_OK) {
        return eSTATUS_FAIL;
    }

    if (pCfg->modes & eSERIAL_PORT_MODE_RX) {
        if (Stm32_Uart_SetupDma (pCfg, pHwCfg, pUartDev, true, pOutSerialPort) != eSTATUS_OK) {
            return eSTATUS_FAIL;
        }
    }

    if (pCfg->modes & eSERIAL_PORT_MODE_TX) {
        if (Stm32_Uart_SetupDma (pCfg, pHwCfg, pUartDev, false, pOutSerialPort) != eSTATUS_OK) {
            return eSTATUS_FAIL;
        }
    }

    // TODO: configure NVIC priority levels
    HAL_NVIC_SetPriority (pHwCfg->irqNum, 8, 8);
    HAL_NVIC_EnableIRQ (pHwCfg->irqNum);

    pOutSerialPort->pSerialHwHandle = (void*)pUartDev;
    pOutSerialPort->pVtbl           = &e_UartVtbl;
    return eSTATUS_OK;
}