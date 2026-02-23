#ifndef DEVICE_SERIAL_SERIAL_H
#define DEVICE_SERIAL_SERIAL_H

#include "hal.h"

#include "core/core.h"

#include "drivers/serial/uart.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

typedef struct {
    UartPort_t port;
    bool const isEnabled;
    bool isInitialized;
} SerialDebug_t;


FJ_DECLARE_SHARED (SerialDebug_t, g_SerialDebug);

static inline SerialDebug_t* SerialDebug_Get (void) {
    if (!g_SerialDebug.isInitialized || !g_SerialDebug.isEnabled) {
        return NULL;
    }
    return &g_SerialDebug;
}

static inline bool SerialDebug_IsEnabled (void) {
    return g_SerialDebug.isEnabled;
}

eSTATUS_t SerialDebug_Init_ (SerialDebug_t* pOutSerial);
static inline eSTATUS_t SerialDebug_Init (void) {
    if (!SerialDebug_IsEnabled ()) {
        return eSTATUS_SUCCESS;
    }
    return SerialDebug_Init_ (&g_SerialDebug);
};

#endif // DEVICE_SERIAL_SERIAL_H