#include "device/serial/serial.h"
#include "conf/conf.h"
#include "core/core.h"
#include "hal.h"
#include "mem/mem.h"
#include "peripheral/bus/bus.h"
#include "target.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>


FJ_DEFINE_SHARED (SerialDebug_t, g_SerialDebug) = {
    .port = {
        .cfg = {
            .id       = BRD_GET_ID(SERIAL_DEBUG, UART),
            .baudRate = 230400U,
        },
    }
};

static void SerialDebugSink (uint8_t const* pData, uint32_t len) {
    UartPort_Write (&g_SerialDebug.port, pData, (uint16_t)len);
}

eSTATUS_t SerialDebugInit_ (SerialDebug_t* pOutSerial) {

    SerialDebug_t* pSerial = pOutSerial;
    eSTATUS_t status       = UartPort_Init (&pSerial->port);
    if (STATUS_FAIL (status)) {
        return eSTATUS_FAILURE;
    }
    LoggerAddSink (SerialDebugSink);
    return eSTATUS_SUCCESS;
}
