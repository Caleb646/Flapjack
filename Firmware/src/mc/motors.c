#include "mc/motors.h"
#include "mc/dshot.h"

#include "core/core.h"

#include <stdbool.h>
#include <stdint.h>

FJ_DEFINE_SHARED (Motors_t, g_Motors) = { 0 };

eSTATUS_t Motors_Init (void) {

    return DShotBB_Init ();
}

eSTATUS_t Motors_Write_ (Motors_t* pMotors, float throttles[BRD_MOTOR_COUNT]) {

    if (!pMotors->armed) {
        LOG_ERROR ("Cannot write to motors because they are not armed");
        return eSTATUS_FAILURE;
    }

    uint16_t dshotThrottles[BRD_MOTOR_COUNT] = { 0 };
    for (uint32_t i = 0; i < BRD_MOTOR_COUNT; ++i) {
        dshotThrottles[i] =
        DSHOT_MIN_THROTTLE +
        (uint16_t)(clipf32 (throttles[i], CFG_MOTOR_MIN_THROTTLE, CFG_MOTOR_MAX_THROTTLE) * (float)DSHOT_RANGE);
    }

    return DShotBB_Write (dshotThrottles);
}

eSTATUS_t Motors_Arm_ (Motors_t* pMotors) {

    uint32_t msDelay                       = 1;
    uint32_t msTotalTimeToArm              = 350;
    uint32_t iterations                    = msTotalTimeToArm / msDelay;
    uint16_t armThrottles[BRD_MOTOR_COUNT] = { 0 };
    for (uint32_t i = 0; i < iterations; ++i) {
        /* NOTE: A DShot value of all 0s is a special command to
         * the esc to arm/disarm the motor depending on the esc's current state.
         * The reason MotorWrite isn't used is because it uses a
         * valid throttle value between > 48 and < 2048 */
        if (DShotBB_Write (armThrottles) != eSTATUS_SUCCESS) {
            LOG_ERROR ("Failed to arm motor");
            return eSTATUS_FAILURE;
        }
        Delay (msDelay);
    }
    pMotors->armed = true;
    LOG_INFO ("Motors armed");
    return eSTATUS_SUCCESS;
}

eSTATUS_t Motors_Disarm_ (Motors_t* pMotors) {

    pMotors->armed = false;
    LOG_INFO ("Motors disarmed");
    return eSTATUS_SUCCESS;
}
