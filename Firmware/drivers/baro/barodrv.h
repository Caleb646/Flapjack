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

#include "drivers/device.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float pressurePa;
    float temperatureC;
} BaroData_t;

typedef struct BaroDriver_s {
    void* ctx;
    eSTATUS_t (*Read) (void* ctx, bool forcePolling, BaroData_t* pOutData);
    bool (*IsDataReady) (void* ctx);
    struct {
        /* Optional. With a Notify set the backend enables the part's data-ready
         * output and wires the EXTI line; zeroed means poll. ODR and
         * oversampling can join it here without changing any call site. */
        DataReadySignal_t signal;
    } cfg;
} BaroDriver_t;

/*
 * Fill pOutDriver->cfg first; this reads it and does NOT clear the struct, the
 * same contract UartPort_Init and SpiDev_Init keep.
 */
eSTATUS_t BaroDrv_Init (BaroDriver_t* pOutDriver);

#endif // DRIVERS_BARO_BARO_DRIVER_H
