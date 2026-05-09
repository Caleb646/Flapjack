#ifndef MC_MOTORS_H
#define MC_MOTORS_H

#include "core/core.h"

#include "target.h"

typedef struct {
    bool armed;
} Motors_t;

FJ_DECLARE_SHARED (Motors_t, g_Motors);

eSTATUS_t Motors_Init (void);
eSTATUS_t Motors_Write_ (Motors_t* pMotors, float throttles[BRD_MOTOR_COUNT]);
static inline eSTATUS_t Motors_Write (float throttles[BRD_MOTOR_COUNT]) {
    return Motors_Write_ (&g_Motors, throttles);
}

eSTATUS_t Motors_Arm_ (Motors_t* pMotors);
static inline eSTATUS_t Motors_Arm (void) {
    return Motors_Arm_ (&g_Motors);
}

eSTATUS_t Motors_Disarm_ (Motors_t* pMotors);
static inline eSTATUS_t Motors_Disarm (void) {
    return Motors_Disarm_ (&g_Motors);
}

#endif // MC_MOTORS_H