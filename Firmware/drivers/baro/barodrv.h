#ifndef DRIVERS_BARO_BARO_DRIVER_H
#define DRIVERS_BARO_BARO_DRIVER_H

/*
 * Generic, device-agnostic barometer driver interface.
 *
 * The application layer (devices/baro.c) talks to the hardware only through the
 * BaroDriver_t vtable below, so a real pressure sensor and a simulation backend
 * are interchangeable. A backend implements `Read`/`IsDataReady` and
 * `BaroDrv_Init`, which binds a BaroDriver_t to its private state.
 *
 * `Read` returns pressure already in pascals and temperature in degrees C -
 * register access, oversampling and the part's own compensation polynomial are
 * backend concerns.
 *
 * This header is the boundary: it must not depend on any backend header.
 */

#include "core/core.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint8_t unused;   // no configurable behaviour yet; keeps the shape of the
                      // other driver interfaces so a real part can add ODR /
                      // oversampling without changing every call site
} BaroDriverConf_t;

typedef struct {
    float pressurePa;
    float temperatureC;
} BaroData_t;

typedef struct BaroDriver_s {
    void* ctx;
    eSTATUS_t (*Read) (void* ctx, bool forcePolling, BaroData_t* pOutData);
    bool (*IsDataReady) (void* ctx);
} BaroDriver_t;

eSTATUS_t BaroDrv_Init (BaroDriverConf_t const* pConf, BaroDriver_t* pOutDriver);

#endif // DRIVERS_BARO_BARO_DRIVER_H
