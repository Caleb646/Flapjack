#ifndef MC_SERVOS_H
#define MC_SERVOS_H

#include <stdint.h>

#include "core/core.h"

#include "target.h"

#include "drivers/servo/servodrv.h"

#define SERVO_LEFT_US_DC   500U
#define SERVO_CENTER_US_DC 1500U
#define SERVO_RIGHT_US_DC  2500U

typedef struct {
    ServoDriver_t drv;
} Servos_t;

FJ_DECLARE_SHARED (Servos_t, g_Servos);

eSTATUS_t Servos_Init (void);
eSTATUS_t Servos_Write (uint16_t const servoVals[BRD_SERVO_COUNT]);

#endif // MC_SERVOS_H
