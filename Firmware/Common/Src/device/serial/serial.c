#include "device/serial/serial.h"
#include "core/core.h"
#include "hal.h"
#include "mem/mem.h"
#include "target.h"


#include "drivers/serial/uart.h"

#include "target.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

// clang-format off
FJ_DEFINE_SHARED (SerialDebug_t, g_SerialDebug) = {
    .port = {
        .cfg = {
            .id       = BRD_GET_UART_ID(SERIAL_DEBUG),
            .baudRate = BRD_GET_BAUD_RATE(SERIAL_DEBUG)
        },
    }
};
// clang-format on

static void SerialDebugSink (uint8_t const* pData, uint32_t len) {
    UartPort_Write (&g_SerialDebug.port, pData, len);
}

eSTATUS_t SerialDebugInit_ (SerialDebug_t* pOutSerial) {

    SerialDebug_t* pSerial = pOutSerial;
    if (STATUS_FAIL (UartPort_Init (&pSerial->port))) {
        return eSTATUS_FAILURE;
    }
    LoggerAddSink (SerialDebugSink);
    return eSTATUS_SUCCESS;
}
