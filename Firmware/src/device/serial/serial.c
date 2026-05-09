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
#if BRD_IS_ENABLED(SERIAL_DEBUG)
    .port = {
        .cfg = {
            .id       = BRD_GET_UART_ID(SERIAL_DEBUG),
            .baudRate = BRD_GET_BAUD_RATE(SERIAL_DEBUG)
        },
    },
    .isEnabled = true,
#endif
};
// clang-format on

static void SerialDebug_Sink (uint8_t const* pData, uint32_t len) {

    UartPort_Write (&g_SerialDebug.port, pData, len);
    for (uint32_t i = 0; i < len; ++i) {
        ITM_SendChar (pData[i]);
    }
}

eSTATUS_t SerialDebug_Init_ (SerialDebug_t* pOutSerial) {

    SerialDebug_t* pSerial = pOutSerial;
    if (STATUS_FAIL (UartPort_Init (&pSerial->port))) {
        return eSTATUS_FAILURE;
    }

    if (STATUS_FAIL (LoggerAddSink (SerialDebug_Sink))) {
        return eSTATUS_FAILURE;
    }

    pOutSerial->isInitialized = true;
    return eSTATUS_SUCCESS;
}
