#ifndef DRIVERS_SERIAL_DEFS_H
#define DRIVERS_SERIAL_DEFS_H

#include <stdint.h>

#include "platform/platform.h"

#include "stl/ring.h"

#include "drivers/dma_defs.h"

#include "drivers/io/gpio_defs.h"

#include "drivers/core/lock.h"

typedef uint8_t eSERIAL_PORT_MODE_t;
enum {
    eSERIAL_PORT_MODE_NONE   = 0,
    eSERIAL_PORT_MODE_TX     = (1U << 0U),
    eSERIAL_PORT_MODE_TX_DMA = (1U << 1U),
    eSERIAL_PORT_MODE_RX     = (1U << 2U),
    eSERIAL_PORT_MODE_RX_DMA = (1U << 3U)
};

typedef uint8_t eSERIAL_PORT_ID_t;

#define eSERIAL_PORT_ID_NONE   0U
#define eSERIAL_PORT_ID_UART_1 1U
#define eSERIAL_PORT_ID_UART_2 2U
#define eSERIAL_PORT_ID_UART_3 3U
#define eSERIAL_PORT_ID_UART_4 4U
#define eSERIAL_PORT_ID_UART_5 5U
#define eSERIAL_PORT_COUNT     5U

// clang-format off
// SERIAL_PORT_ID_MAKE(UART_1) expands to eSERIAL_PORT_ID_UART_1
#define SERIAL_PORT_ID_EXPAND(PORTNAME) eSERIAL_PORT_ID_##PORTNAME
#define SERIAL_PORT_ID_MAKE(...) SERIAL_PORT_ID_EXPAND(__VA_ARGS__) 

#define SERIAL_PORT_ID_VALID(PORT_ID) ((PORT_ID) != eSERIAL_PORT_ID_NONE && (PORT_ID) <= eSERIAL_PORT_COUNT)
#define SERIAL_PORT_ID_TO_INDEX(PORT_ID) ((PORT_ID) - 1U)
// clang-format on

typedef uint32_t eSERIAL_PORT_BAUD_t;
enum {
    eSERIAL_PORT_BAUD_9600   = 9600,
    eSERIAL_PORT_BAUD_19200  = 19200,
    eSERIAL_PORT_BAUD_38400  = 38400,
    eSERIAL_PORT_BAUD_57600  = 57600,
    eSERIAL_PORT_BAUD_115200 = 115200,
    eSERIAL_PORT_BAUD_230400 = 230400,
    eSERIAL_PORT_BAUD_460800 = 460800,
    eSERIAL_PORT_BAUD_921600 = 921600
};

typedef uint16_t eSERIAL_PORT_FUNCTION_t;
enum {
    eSERIAL_PORT_FUNCTION_NULL      = 0,
    eSERIAL_PORT_FUNCTION_RX_SERIAL = (1U << 0U),
    eSERIAL_PORT_FUNCTION_TX_SERIAL = (1U << 1U),
    eSERIAL_PORT_FUNCTION_GPS       = (1U << 2U)
};

typedef uint8_t eSERIAL_PORT_TYPE_t;
enum {
    eSERIAL_PORT_TYPE_NULL = 0,
    eSERIAL_PORT_TYPE_UART,
    eSERIAL_PORT_TYPE_USB_VCP


};

typedef struct SerialPortCfg_s {

    eSERIAL_PORT_ID_t portId;
    eSERIAL_PORT_BAUD_t baudrate;
    eSERIAL_PORT_TYPE_t portType;
    uint16_t modes;
    uint16_t functions;
    eIRQ_PRIO_t irqPrior;

    uint32_t rxBufferSize;
    uint32_t txBufferSize;

    bool isShared;

} SerialPortCfg_t;

typedef struct SerialPort_s SerialPort_t;
typedef void (*SerialRxCallback_t) (SerialPort_t* pPort, uint8_t const* pData, uint32_t size, void* pArg);

typedef struct SerialPortVtable_s {
    eSTATUS_t (*fnWrite) (SerialPort_t* pSerialPort, uint8_t const* pData, uint32_t size);
    uint32_t (*fnRead) (SerialPort_t* pSerialPort, uint8_t* pData, uint32_t size);
    eSTATUS_t (*fnSetBaud) (SerialPort_t* pSerialPort, eSERIAL_PORT_BAUD_t baudrate);
    // eSTATUS_t (*fnSetRxCallback) (SerialPort_t* pSerialPort, SerialRxCallback_t fnCallback, void* pCallbackArg);
} SerialPortVtable_t;

typedef struct SerialPort_s {

    eSERIAL_PORT_ID_t portId;
    eSERIAL_PORT_BAUD_t baudrate;
    uint16_t modes;
    uint16_t functions;

    uint32_t irqNum;
    uint32_t irqPrior;
    bool isShared;

    void* pSerialHwHandle;

    SerialRxCallback_t fnRxCallback;
    void* pRxCallbackArg;

    SerialPortVtable_t* pVtbl;

    bool volatile isRxBusy;
    RingBuff_t volatile rxRingBuff;
    SpinLock_t rxLock;

    bool volatile isTxBusy;
    uint32_t volatile txBytesInProgress;
    RingBuff_t volatile txRingBuff;
    SpinLock_t txLock;

    DmaDevice_t* pRxDmaDev;
    DmaDevice_t* pTxDmaDev;

} SerialPort_t;

typedef struct UartHwCfg_s {

    eSERIAL_PORT_ID_t portId;
    USART_TypeDef* pInstance;

    uint32_t volatile* pRccReg;
    uint32_t rccMask;

    eGPIO_ID_t txOpts[PLAT_UART_MAX_PIN_SEL];
    eGPIO_ID_t rxOpts[PLAT_UART_MAX_PIN_SEL];
    uint8_t afOpts[PLAT_UART_MAX_PIN_SEL];

    IRQn_Type irqNum;

    uint32_t dma_RxRequestId;
    uint32_t dma_TxRequestId;

} UartHwCfg_t;

typedef struct UartDevice_s {

    eSERIAL_PORT_ID_t portId;
    UART_HandleTypeDef handle;
    SerialPort_t* pSerialPort;

} UartDevice_t;

#endif // DRIVERS_SERIAL_DEFS_H