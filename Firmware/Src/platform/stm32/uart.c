#include <stdbool.h>
#include <stdint.h>

#include "common.h"

#include "drivers/serial/serial.h"

#include "drivers/io/gpio.h"
#include "drivers/io/gpio_defs.h"

#include "platform/platform.h"

#include "targets/target.h"

typedef struct UartHwCfg_s {
    USART_TypeDef* pInstance;
    uint32_t volatile* pRccReg;
    uint32_t rccMask;
    eGPIO_ID_t txOpts[PLAT_UART_MAX_PIN_SEL];
    eGPIO_ID_t rxOpts[PLAT_UART_MAX_PIN_SEL];
    uint8_t afOpts[PLAT_UART_MAX_PIN_SEL];
    IRQn_Type irqNum;
} UartHwCfg_t;

typedef struct UartDevice_s {
    UART_HandleTypeDef handle;
    eSERIAL_PORT_ID_t portId;
} UartDevice_t;

FJ_DEFINE_SHARED (UartDevice_t*, g_pUartDevices[PLAT_UART_MAX_DEVS]);

FJ_TESTABLE eSTATUS_t Stm32_Uart_Write (SerialPort_t* pSerialPort, const uint8_t* pData, uint32_t nData);
FJ_TESTABLE eSTATUS_t Stm32_Uart_Read (SerialPort_t* pSerialPort, uint8_t* pData, uint32_t nData);
FJ_TESTABLE eSTATUS_t Stm32_Uart_SetBaud (SerialPort_t* pSerialPort, eSERIAL_PORT_BAUD_t baudrate);

FJ_DEFINE_SHARED (SerialPortVtable_t, g_Stm32_Uart_Vtable) = {
    .fnWrite   = Stm32_Uart_Write,
    .fnRead    = Stm32_Uart_Read,
    .fnSetBaud = Stm32_Uart_SetBaud,
};

static uint16_t Stm32_Uart_PortIdToDevIndex (eSERIAL_PORT_ID_t portId) {

    switch (portId) {
    case eSERIAL_PORT_ID_UART1: return 0U;
    case eSERIAL_PORT_ID_UART2: return 1U;
    case eSERIAL_PORT_ID_UART3: return 2U;
    case eSERIAL_PORT_ID_UART4: return 3U;
    case eSERIAL_PORT_ID_UART5: return 4U;
    default: return UINT16_MAX;
    }
}

static UartDevice_t* Stm32_Uart_Alloc (eSERIAL_PORT_ID_t portId) {

    uint16_t devIndex = Stm32_Uart_PortIdToDevIndex (portId);
    if (devIndex >= PLAT_UART_MAX_DEVS) {
        return NULL;
    }

    UartDevice_t* pUartDev = (UartDevice_t*)Alloc_SharedMem (sizeof (UartDevice_t));
    if (!pUartDev) {
        return NULL;
    }

    pUartDev->portId = portId;

    g_pUartDevices[devIndex] = pUartDev;
    return pUartDev;
}

static UartDevice_t* Stm32_Uart_GetByPortId (eSERIAL_PORT_ID_t portId) {

    uint16_t devIndex = Stm32_Uart_PortIdToDevIndex (portId);
    if (devIndex >= PLAT_UART_MAX_DEVS) {
        return NULL;
    }
    return g_pUartDevices[devIndex];
}

static UartHwCfg_t Stm32_Uart_GetHwCfgByPortId (eSERIAL_PORT_ID_t portId) {

    switch (portId) {
    case eSERIAL_PORT_ID_UART1:
        return (UartHwCfg_t){
            .pInstance = USART1,
            .pRccReg   = &RCC->APB2ENR,
            .rccMask   = RCC_APB2ENR_USART1EN,
            .txOpts    = { PLAT_GPIO_ID_MAKE (A, 9) },
            .rxOpts    = { PLAT_GPIO_ID_MAKE (A, 10) },
            .afOpts    = { GPIO_AF7_USART1 },
            .irqNum    = USART1_IRQn,
        };
    case eSERIAL_PORT_ID_UART2:
        return (UartHwCfg_t){
            .pInstance = USART2,
            .pRccReg   = &RCC->APB1LENR,
            .rccMask   = RCC_APB1LENR_USART2EN,
            .txOpts    = { PLAT_GPIO_ID_MAKE (A, 2) },
            .rxOpts    = { PLAT_GPIO_ID_MAKE (A, 3) },
            .afOpts    = { GPIO_AF7_USART2 },
            .irqNum    = USART2_IRQn,
        };
    }
    return (UartHwCfg_t){ 0 };
}

FJ_TESTABLE eSTATUS_t Stm32_Uart_Write (SerialPort_t* pSerialPort, const uint8_t* pData, uint32_t nData) {

    UartDevice_t* pUartDev = (UartDevice_t*)pSerialPort->pSerialHwHandle;
    if (!pUartDev) {
        return eSTATUS_FAIL;
    }

    if (HAL_UART_Transmit (&pUartDev->handle, (uint8_t*)pData, nData, HAL_MAX_DELAY) != HAL_OK) {
        return eSTATUS_FAIL;
    }
    return eSTATUS_OK;
}

FJ_TESTABLE eSTATUS_t Stm32_Uart_Read (SerialPort_t* pSerialPort, uint8_t* pData, uint32_t nData) {

    UartDevice_t* pUartDev = (UartDevice_t*)pSerialPort->pSerialHwHandle;
    if (!pUartDev) {
        return eSTATUS_FAIL;
    }

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
        pUartDev = Stm32_Uart_Alloc (pCfg->portId);
        if (!pUartDev) {
            return eSTATUS_FAIL;
        }
    } else {
        return eSTATUS_OK; // already initialized
    }

    UartHwCfg_t hwCfg = Stm32_Uart_GetHwCfgByPortId (pCfg->portId);
    if (!hwCfg.pInstance) {
        return eSTATUS_FAIL;
    }

    eGPIO_ID_t txPin = 0U;
    eGPIO_ID_t rxPin = 0U;
    switch (pCfg->portType) {
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
        if (hwCfg.txOpts[i] == txPin && hwCfg.rxOpts[i] == rxPin) {
            GPIO_Init (txPin, pCfg->portId, PLAT_GPIO_CFG_UART, hwCfg.afOpts[i]);
            GPIO_Init (rxPin, pCfg->portId, PLAT_GPIO_CFG_UART, hwCfg.afOpts[i]);
            found = true;
            break;
        }
    }

    if (!found) {
        return eSTATUS_FAIL;
    }

    *(hwCfg.pRccReg) |= hwCfg.rccMask; // enable rcc
    pUartDev->handle.Instance                    = hwCfg.pInstance;
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

    pOutSerialPort->pSerialHwHandle = (void*)pUartDev;
    pOutSerialPort->pVtbl           = &g_Stm32_Uart_Vtable;
    return eSTATUS_OK;
}