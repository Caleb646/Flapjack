#ifndef DEVICE_SERIAL_SERIAL_H
#define DEVICE_SERIAL_SERIAL_H

#include "hal.h"

#include "core/core.h"

#include "drivers/serial/uart.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

typedef struct {
    UartPort_t port;
} SerialDebug_t;


FJ_DECLARE_SHARED (SerialDebug_t, g_SerialDebug);

eSTATUS_t SerialDebugInit_ (SerialDebug_t* pOutSerial);
static inline eSTATUS_t SerialDebugInit (void) {
    return SerialDebugInit_ (&g_SerialDebug);
};

#endif // DEVICE_SERIAL_SERIAL_H