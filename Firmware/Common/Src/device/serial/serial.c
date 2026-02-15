#include "hal.h"
#include "target.h"

#include "core/core.h"

#include "drivers/serial/uart.h"

#include "device/serial/serial.h"

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

    for (uint32_t i = 0; i < len; ++i) {
        ITM_SendChar (pData[i]);
    }
}

eSTATUS_t SerialDebugInit_ (SerialDebug_t* pOutSerial) {

    SerialDebug_t* pSerial = pOutSerial;
    if (STATUS_FAIL (UartPort_Init (&pSerial->port))) {
        return eSTATUS_FAILURE;
    }

    return LoggerAddSink (SerialDebugSink);
}
