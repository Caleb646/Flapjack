#ifndef DRIVERS_SERIAL_DEFS_H
#define DRIVERS_SERIAL_DEFS_H

#include <stdint.h>

typedef uint8_t eSERIAL_PORT_MODE_t;
enum {
    eSERIAL_PORT_MODE_NULL = 0,
    eSERIAL_PORT_MODE_TX,
    eSERIAL_PORT_MODE_RX,
    eSERIAL_PORT_MODE_TX_RX
};

typedef uint8_t eSERIAL_PORT_ID_t;
enum {
    eSERIAL_PORT_ID_NULL = 0,
    eSERIAL_PORT_ID_UART1,
    eSERIAL_PORT_ID_UART2,
    eSERIAL_PORT_ID_UART3,
    eSERIAL_PORT_ID_UART4,
    eSERIAL_PORT_ID_UART5,
    eSERIAL_PORT_COUNT = eSERIAL_PORT_ID_UART5
};
// clang-format off
#define SERIAL_PORT_ID_VALID(PORT_ID) ((PORT_ID) != eSERIAL_PORT_ID_NULL && (PORT_ID) < eSERIAL_PORT_COUNT)
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
    eSERIAL_PORT_FUNCTION_t functions;
} SerialPortCfg_t;

typedef struct SerialPort_s SerialPort_t;
typedef void (*SerialRxCallback_t) (SerialPort_t* pPort, uint8_t const* pData, uint32_t size, void* pArg);

typedef struct SerialPortVtable_s {
    eSTATUS_t (*fnWrite) (SerialPort_t* pSerialPort, uint8_t const* pData, uint32_t size);
    eSTATUS_t (*fnRead) (SerialPort_t* pSerialPort, uint8_t* pData, uint32_t size);
    eSTATUS_t (*fnSetBaud) (SerialPort_t* pSerialPort, eSERIAL_PORT_BAUD_t baudrate);
} SerialPortVtable_t;

typedef struct SerialPort_s {
    eSERIAL_PORT_ID_t portId;
    eSERIAL_PORT_MODE_t mode;
    eSERIAL_PORT_BAUD_t baudrate;
    uint16_t functions;

    void* pSerialHwHandle;

    SerialRxCallback_t fnRxCallback;
    void* pRxCallbackArg;

    SerialPortVtable_t* pVtbl;
} SerialPort_t;


#endif // DRIVERS_SERIAL_DEFS_H