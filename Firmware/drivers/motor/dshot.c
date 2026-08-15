/*
 * DShot motor backend (real hardware). The throttle->DShot mapping and the
 * ESC arming handshake moved here verbatim from the old devices/motors.c so the
 * device layer can select it through MotorDriver_t.
 */

#include "target.h"

#include "core/core.h"

#include "drivers/motor/motordrv.h"
#include "drivers/dshot/dshot.h"

#include <stdint.h>
#include <string.h>

static eSTATUS_t Dshot_Write (void* ctx, float const throttles[BRD_MOTOR_COUNT]) {

    FJ_UNUSED (ctx);
    uint16_t dshotThrottles[BRD_MOTOR_COUNT] = { 0 };
    for (uint32_t i = 0; i < BRD_MOTOR_COUNT; ++i) {
        dshotThrottles[i] =
        DSHOT_MIN_THROTTLE +
        (uint16_t)(clipf32 (throttles[i], CFG_MOTOR_MIN_THROTTLE, CFG_MOTOR_MAX_THROTTLE) * (float)DSHOT_RANGE);
    }
    return DShotBB_Write (dshotThrottles);
}

static eSTATUS_t Dshot_Arm (void* ctx) {

    FJ_UNUSED (ctx);
    uint32_t msDelay                       = 1;
    uint32_t msTotalTimeToArm              = 350;
    uint32_t iterations                    = msTotalTimeToArm / msDelay;
    uint16_t armThrottles[BRD_MOTOR_COUNT] = { 0 };
    for (uint32_t i = 0; i < iterations; ++i) {
        /* NOTE: A DShot value of all 0s is a special command to
         * the esc to arm/disarm the motor depending on the esc's current state. */
        if (DShotBB_Write (armThrottles) != eSTATUS_SUCCESS) {
            return eSTATUS_FAILURE;
        }
        Delay (msDelay);
    }
    return eSTATUS_SUCCESS;
}

static eSTATUS_t Dshot_Disarm (void* ctx) {
    FJ_UNUSED (ctx);
    return eSTATUS_SUCCESS;
}

eSTATUS_t MotorDrv_Init (MotorDriver_t* pOutDriver) {

    if (!pOutDriver) {
        return eSTATUS_NULL_ARG;
    }
    memset (pOutDriver, 0, sizeof (MotorDriver_t));

    eSTATUS_t status = DShotBB_Init ();
    if (STATUS_FAIL (status)) {
        return status;
    }

    pOutDriver->ctx    = NULL;
    pOutDriver->Write  = Dshot_Write;
    pOutDriver->Arm    = Dshot_Arm;
    pOutDriver->Disarm = Dshot_Disarm;
    return eSTATUS_SUCCESS;
}
