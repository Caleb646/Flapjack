#ifndef PERIPHS_UART_H
#define PERIPHS_UART_H
#include "common.h"
#include "conf/board.h"
#include "conf/conf.h"
#include "hal.h"
#include "peripheral/gpio.h"

typedef uint32_t eUART_BAUD_t;
enum {
    eUART_BAUD_9600   = 9600U,
    eUART_BAUD_19200  = 19200U,
    eUART_BAUD_38400  = 38400U,
    eUART_BAUD_57600  = 57600U,
    eUART_BAUD_115200 = 115200U,
    eUART_BAUD_230400 = 230400U
};

typedef uint8_t eUART_CALLBACK_ID_t;
enum {
    eUART_CALLBACK_ID_RX = 0,
    eUART_CALLBACK_ID_TX,
    eUART_CALLBACK_ID_ERROR,
    eUART_CALLBACK_ID_MAX
};

// typedef struct UARTBus_t UARTBus_t;
// Callback function type
typedef void (*UART_Callback_t) (eBUS_ID_t busId);
typedef struct {
    eDEVICE_ID_t deviceId;
} UARTInitConf_t;

typedef struct {
    UART_HandleTypeDef handle;
    eBUS_ID_t busId;
    eDEVICE_ID_t deviceId;
    bool isInitialized;
    UART_Callback_t rxCallback;
    UART_Callback_t txCallback;
    UART_Callback_t errorCallback;
} UARTBus_t;

// typedef UARTBus_t volatile vUARTBus_t;
typedef UARTBus_t vUARTBus_t;

eSTATUS_t UARTInit (UARTInitConf_t conf, UARTBoardConf_t boardConf);
eSTATUS_t UARTRead_Blocking (eBUS_ID_t busId, uint8_t* pData, uint32_t size);
eSTATUS_t UARTWrite_Blocking (eBUS_ID_t busId, uint8_t const* pData, uint32_t size);
eSTATUS_t UARTRead_IT (eBUS_ID_t busId, uint8_t* pData, uint32_t size);
eSTATUS_t UARTRegisterCallback (eBUS_ID_t busId, eUART_CALLBACK_ID_t cbId, UART_Callback_t callback);
eSTATUS_t UARTEnableInterrupts (eBUS_ID_t busId, uint32_t priority);
bool UARTIsValid (eBUS_ID_t busId);

#define UART_INIT_FROM_BOARD_CONF(pSTATUS, DEVICE_BOARD_CONF, UART_BOARD_CONF) \
    do {                                                                       \
        UARTInitConf_t conf = { 0 };                                           \
        conf.deviceId       = (DEVICE_BOARD_CONF).deviceId;                    \
        *(pSTATUS)          = UARTInit (conf, (UART_BOARD_CONF));              \
    } while (0)

#endif // PERIPHS_UART_H
