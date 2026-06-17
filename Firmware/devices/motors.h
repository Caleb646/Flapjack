#ifndef MC_MOTORS_H
#define MC_MOTORS_H

#include "core/core.h"

#include "target.h"

typedef struct {
    bool armed;
} Motors_t;

eSTATUS_t Motors_Init (Motors_t* pOutMotors);
eSTATUS_t Motors_Write (Motors_t* pMotors, float throttles[BRD_MOTOR_COUNT]);
eSTATUS_t Motors_Arm (Motors_t* pMotors);
eSTATUS_t Motors_Disarm (Motors_t* pMotors);

#endif // MC_MOTORS_H