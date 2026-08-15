#ifndef MC_MOTORS_H
#define MC_MOTORS_H

#include <stdbool.h>

#include "core/core.h"

#include "target.h"

#include "drivers/motor/motordrv.h"

typedef struct {
    MotorDriver_t drv;
    bool armed;
} Motors_t;

eSTATUS_t Motors_Init (Motors_t* pOutMotors);
eSTATUS_t Motors_Write (Motors_t* pMotors, float throttles[BRD_MOTOR_COUNT]);
eSTATUS_t Motors_Arm (Motors_t* pMotors);
eSTATUS_t Motors_Disarm (Motors_t* pMotors);

#endif // MC_MOTORS_H
