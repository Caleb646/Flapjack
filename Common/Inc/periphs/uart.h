#ifndef PERIPHS_UART_H
#define PERIPHS_UART_H
#include "common.h"
#include "conf.h"
#include "hal.h"
#include "periphs/gpio.h"

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
typedef void (*UART_Callback_t) (eUART_BUS_ID_t busId);
typedef struct {
    eUART_BUS_ID_t busId;
    eDEVICE_ID_t deviceId;
    eUART_BAUD_t baudRate;
} UARTInitConf_t;

typedef struct {
    UART_HandleTypeDef handle;
    GPIOUART_t gpio;
    eUART_BUS_ID_t busId;
    eUART_BAUD_t baudRate;
    eDEVICE_ID_t deviceId;
    BOOL_t isInitialized;
    UART_Callback_t rxCallback;
    UART_Callback_t txCallback;
    UART_Callback_t errorCallback;
} UARTBus_t;

eSTATUS_t UARTInit (UARTInitConf_t const* pConf);
eSTATUS_t UARTRead_Blocking (eUART_BUS_ID_t busId, uint8_t* pData, uint16_t size);
eSTATUS_t UARTWrite_Blocking (eUART_BUS_ID_t busId, uint8_t const* pData, uint16_t size);
eSTATUS_t UARTRead_IT (eUART_BUS_ID_t busId, uint8_t* pData, uint16_t size);
eSTATUS_t UARTRegisterCallback (eUART_BUS_ID_t busId, eUART_CALLBACK_ID_t cbId, UART_Callback_t callback);
eSTATUS_t UARTEnableInterrupt (eUART_BUS_ID_t busId, uint32_t priority);
BOOL_t UARTIsValid (eUART_BUS_ID_t busId);

eSTATUS_t UARTSystemInit (void);
eSTATUS_t UARTSystemEnableInterrupts (void);

#endif // PERIPHS_UART_H
