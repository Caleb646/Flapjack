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
    DeviceBoardConf_t deviceBoardConf;
    BusBoardConf_t busBoardConf;
} UARTInitConf_t;

typedef struct {
    eBUS_ID_t busId;
    eDEVICE_ID_t deviceId;
    UART_HandleTypeDef handle;
    UART_Callback_t rxCallback;
    UART_Callback_t txCallback;
    UART_Callback_t errorCallback;
    bool isInitialized;
} UARTBus_t;

// typedef UARTBus_t volatile vUARTBus_t;
typedef UARTBus_t vUARTBus_t;

// clang-format off
vUARTBus_t* UARTGetBusById (eBUS_ID_t busId);
eSTATUS_t UARTInit (UARTInitConf_t conf, vUARTBus_t* pOutBus);
eSTATUS_t UARTRead_Blocking (vUARTBus_t* pBus, eDEVICE_ID_t deviceId, uint8_t* pData, size_t size);
eSTATUS_t UARTWrite_Blocking (vUARTBus_t* pBus, eDEVICE_ID_t deviceId, uint8_t const* pData, size_t size);
eSTATUS_t UARTRead_IT (vUARTBus_t* pBus, eDEVICE_ID_t deviceId, uint8_t* pData, size_t size);
eSTATUS_t UARTRegisterCallback (vUARTBus_t* pBus, eUART_CALLBACK_ID_t cbId, UART_Callback_t callback);
eSTATUS_t UARTEnableInterrupts (vUARTBus_t* pBus, uint32_t priority);

#define UART_INIT(pSTATUS, DEVICE_BOARD_CONF, UART_BOARD_CONF) \
    do {                                                       \
        UARTInitConf_t conf  = { 0 };                          \
        conf.deviceBoardConf = (DEVICE_BOARD_CONF);            \
        conf.busBoardConf    = (UART_BOARD_CONF);              \
        *(pSTATUS)           = UARTInit (conf, NULL);          \
    } while (0)

eSTATUS_t UART_READ_BLOCKING(void* pCtx, eDEVICE_ID_t deviceId, uint8_t* pData, size_t size);
eSTATUS_t UART_WRITE_BLOCKING(void* pCtx, eDEVICE_ID_t deviceId, uint8_t const* pData, size_t size);
eSTATUS_t UART_READ_IT(void* pCtx, eDEVICE_ID_t deviceId, uint8_t* pData, size_t size);

#define UART_REGISTER_CALLBACK(BUS_ID, CB_ID, CALLBACK) \
    UARTRegisterCallback (UARTGetBusById (BUS_ID), (CB_ID), (CALLBACK))

#define UART_ENABLE_INTERRUPTS(BUS_ID, PRIORITY) \
    UARTEnableInterrupts (UARTGetBusById (BUS_ID), (PRIORITY))

// clang-format on

#endif // PERIPHS_UART_H
