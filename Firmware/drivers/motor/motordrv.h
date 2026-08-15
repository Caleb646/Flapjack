#ifndef DRIVERS_MOTOR_MOTOR_DRIVER_H
#define DRIVERS_MOTOR_MOTOR_DRIVER_H

/*
 * Generic, device-agnostic motor (ESC) driver interface.
 *
 * The device layer (devices/motors.c) drives motors only through the
 * MotorDriver_t vtable below, so a DShot backend and a simulation backend are
 * interchangeable at compile time (selected by the driver profile), like the
 * IMU/mag drivers.
 *
 * `Write` takes per-motor throttle in [0.0, 1.0]. `Arm`/`Disarm` perform any
 * ESC handshake; the device layer owns the armed flag.
 *
 * This header is the boundary: it must not depend on any backend header.
 */

#include "core/core.h"

#include "target.h"

typedef struct MotorDriver_s {
    void* ctx;
    eSTATUS_t (*Write) (void* ctx, float const throttles[BRD_MOTOR_COUNT]);
    eSTATUS_t (*Arm) (void* ctx);
    eSTATUS_t (*Disarm) (void* ctx);
} MotorDriver_t;

eSTATUS_t MotorDrv_Init (MotorDriver_t* pOutDriver);

#endif // DRIVERS_MOTOR_MOTOR_DRIVER_H
