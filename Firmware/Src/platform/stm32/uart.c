#include <stdbool.h>
#include <stdint.h>

#include "common.h"

#include "drivers/serial/serial.h"

#include "drivers/io/gpio.h"
#include "drivers/io/gpio_defs.h"

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

void Stm32_UART_IRQHandler (eSERIAL_PORT_ID_t portId) {

    UartDevice_t* pUartDev = Stm32_Uart_GetByPortId (portId);
    if (!pUartDev) {
        return;
    }

    HAL_UART_IRQHandler (&pUartDev->handle);
}

#define UART_IRQ_HANDLER_DEF(TYPE, NUM, PORT_ID)               \
    void TYPE##NUM##_IRQHandler (void) {                       \
        Stm32_UART_IRQHandler (eSERIAL_PORT_ID_UART##PORT_ID); \
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

FJ_TESTABLE eSTATUS_t Stm32_Uart_Write (SerialPort_t* pSerialPort, const uint8_t* pData, uint32_t nData) {

    UartDevice_t* pUartDev = (UartDevice_t*)pSerialPort->pSerialHwHandle;
    if (HAL_UART_Transmit (&pUartDev->handle, (uint8_t*)pData, nData, HAL_MAX_DELAY) != HAL_OK) {
        return eSTATUS_FAIL;
    }
    return eSTATUS_OK;
}

FJ_TESTABLE eSTATUS_t Stm32_Uart_Read (SerialPort_t* pSerialPort, uint8_t* pData, uint32_t nData) {

    UartDevice_t* pUartDev = (UartDevice_t*)pSerialPort->pSerialHwHandle;
    if (HAL_UART_Receive (&pUartDev->handle, pData, nData, HAL_MAX_DELAY) != HAL_OK) {
        return eSTATUS_FAIL;
    }
    return eSTATUS_OK;
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

    if (HAL_UART_Init (&pUartDev->handle) != HAL_OK) {
        return eSTATUS_FAIL;
    }

    // TODO: configure NVIC priority levels
    HAL_NVIC_SetPriority (pHwCfg->irqNum, 8, 8);
    HAL_NVIC_EnableIRQ (pHwCfg->irqNum);


    pOutSerialPort->pSerialHwHandle = (void*)pUartDev;
    pOutSerialPort->pVtbl           = &e_UartVtbl;
    return eSTATUS_OK;
}