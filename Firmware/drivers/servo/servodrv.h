#ifndef DRIVERS_SERVO_SERVO_DRIVER_H
#define DRIVERS_SERVO_SERVO_DRIVER_H

/*
 * Generic, device-agnostic servo output driver interface.
 *
 * The device layer (devices/servos.c) drives servos only through the
 * ServoDriver_t vtable below, so a PWM backend and a simulation backend are
 * interchangeable at compile time (selected by the driver profile), exactly
 * like the IMU/mag drivers.
 *
 * `Write` takes per-servo pulse widths in microseconds (the native PWM unit);
 * the sim backend reinterprets these as a tilt angle and forwards them over the
 * sim link.
 *
 * This header is the boundary: it must not depend on any backend header.
 */

#include "core/core.h"

#include "target.h"

#include <stdint.h>

typedef struct ServoDriver_s {
    void* ctx;
    eSTATUS_t (*Write) (void* ctx, uint16_t const us[BRD_SERVO_COUNT]);
} ServoDriver_t;

eSTATUS_t ServoDrv_Init (ServoDriver_t* pOutDriver);

#endif // DRIVERS_SERVO_SERVO_DRIVER_H
