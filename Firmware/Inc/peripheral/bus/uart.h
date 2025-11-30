#ifndef PERIPHS_UART_H
#define PERIPHS_UART_H
#include "conf/board.h"
#include "conf/conf.h"
#include "core/core.h"
#include "hal.h"
#include "peripheral/bus/common.h"
#include "peripheral/gpio.h"


typedef uint8_t eUART_BAUD_t;
enum {
    eUART_BAUD_9600 = 0,
    eUART_BAUD_19200,
    eUART_BAUD_38400,
    eUART_BAUD_57600,
    eUART_BAUD_115200,
    eUART_BAUD_230400,
};

typedef struct {
    BusDesc_t* pBusDesc;
} UARTInitConf_t;

typedef struct {
    eBUS_ID_t busId;
    eDEVICE_ID_t deviceId;
    UART_HandleTypeDef handle;
    BusCallback_t callbacks[eBUS_NUMBER_OF_CALLBACK_IDS];
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
eSTATUS_t UARTRegisterCallback (vUARTBus_t* pBus, eDEVICE_ID_t deviceId, BusCallback_t callback);
eSTATUS_t UARTEnableInterrupts (vUARTBus_t* pBus, uint32_t priority);
// clang-format on

#define UART_INIT(pBUSDESC) UARTInit ((UARTInitConf_t){ .pBusDesc = (pBUSDESC) }, NULL)

#endif // PERIPHS_UART_H
