/*
 * Simulation magnetometer backend: returns the latest normalized field that the
 * JSBSim bridge synthesized from vehicle attitude and delivered over the sim
 * link. Non-blocking (Nav peeks mag); before the first sample arrives it falls
 * back to a north-pointing field.
 */

#include "drivers/mag/magdrv.h"

#include "core/core.h"

#include "drivers/sim_link/sim_link.h"

#include <string.h>
#include <stdbool.h>

STATIC eSTATUS_t Sim_Read (void* ctx, bool forcePolling, Vec3f* pField) {

    FJ_UNUSED (forcePolling);
    FJ_UNUSED (ctx);
    if (!pField) {
        return eSTATUS_FAILURE;
    }

    float mag[3];
    if (!SimLink_GetMag (mag)) {
        /* No sample yet: north-pointing field (normalised). */
        pField->x = 1.0F;
        pField->y = 0.0F;
        pField->z = 0.0F;
        return eSTATUS_SUCCESS;
    }

    pField->x = mag[0];
    pField->y = mag[1];
    pField->z = mag[2];
    return eSTATUS_SUCCESS;
}

STATIC bool Sim_IsDataReady (void* ctx) {
    FJ_UNUSED (ctx);
    return true;
}

eSTATUS_t MagDrv_Init (MagDriverConf_t const* pConf, MagDriver_t* pOutDriver) {

    if (!pConf || !pOutDriver) {
        return eSTATUS_NULL_ARG;
    }

    memset (pOutDriver, 0, sizeof (MagDriver_t));
    pOutDriver->ctx         = NULL;
    pOutDriver->Read        = Sim_Read;
    pOutDriver->IsDataReady = Sim_IsDataReady;
    return eSTATUS_SUCCESS;
}
