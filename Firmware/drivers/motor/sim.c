/*
 * Simulation motor backend: instead of driving DShot ESCs, stream each motor's
 * throttle (0.0 - 1.0) to the JSBSim bridge over the sim link. Arm/Disarm are
 * no-ops (there is no ESC handshake in sim).
 */

#include "core/core.h"

#include "target.h"

#include "drivers/motor/motordrv.h"
#include "drivers/sim_link/sim_link.h"

#include <string.h>

STATIC eSTATUS_t Sim_Write (void* ctx, float const throttles[BRD_MOTOR_COUNT]) {
    FJ_UNUSED (ctx);
    return SimLink_SendThrottles (throttles, BRD_MOTOR_COUNT);
}

STATIC eSTATUS_t Sim_Arm (void* ctx) {
    FJ_UNUSED (ctx);
    return eSTATUS_SUCCESS;
}

STATIC eSTATUS_t Sim_Disarm (void* ctx) {
    FJ_UNUSED (ctx);
    return eSTATUS_SUCCESS;
}

eSTATUS_t MotorDrv_Init (MotorDriver_t* pOutDriver) {

    if (!pOutDriver) {
        return eSTATUS_NULL_ARG;
    }
    memset (pOutDriver, 0, sizeof (MotorDriver_t));
    pOutDriver->ctx    = NULL;
    pOutDriver->Write  = Sim_Write;
    pOutDriver->Arm    = Sim_Arm;
    pOutDriver->Disarm = Sim_Disarm;
    return eSTATUS_SUCCESS;
}
