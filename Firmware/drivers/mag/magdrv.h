#ifndef DRIVERS_MAG_MAG_DRIVER_H
#define DRIVERS_MAG_MAG_DRIVER_H

/*
 * Generic, device-agnostic magnetometer driver interface.
 *
 * The application layer (device/mag.c) talks to the hardware only through the
 * MagDriver_t vtable below, so an MMC5983 backend and a simulation backend are
 * interchangeable. A backend implements `Read`/`IsDataReady` and `MagDrv_Init`,
 * which binds a MagDriver_t to its private state.
 *
 * `Read` returns the magnetic field already scaled by the backend (normalised to
 * full-scale when conf.normalize is set) - register access and scaling are
 * backend concerns.
 *
 * This header is the boundary: it must not depend on any backend header.
 */

#include "core/core.h"

#include "drivers/device.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    Vec3f field;
} MagData_t;

typedef struct MagDriver_s {
    void* ctx;
    eSTATUS_t (*Read) (void* ctx, bool forcePolling, MagData_t* pOutData);
    bool (*IsDataReady) (void* ctx);
    struct {
        bool normalize;   // normalise field to full-scale [-1, 1]
        /* Optional. With a Notify set the backend enables the part's
         * measurement-done output and wires the EXTI line; zeroed means poll. */
        DataReadySignal_t signal;
    } cfg;
} MagDriver_t;

/*
 * Fill pOutDriver->cfg first; this reads it and does NOT clear the struct, the
 * same contract UartPort_Init and SpiDev_Init keep.
 */
eSTATUS_t MagDrv_Init (MagDriver_t* pOutDriver);

#endif // DRIVERS_MAG_MAG_DRIVER_H
