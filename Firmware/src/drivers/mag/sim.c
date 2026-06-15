
#include "drivers/mag/magdrv.h"

#include "core/core.h"

#include <string.h>
#include <stdbool.h>

typedef struct SimMag_s {
    uint8_t unused;
} SimMag_t;

STATIC eSTATUS_t Sim_Read (void* ctx, bool forcePolling, Vec3f* pField) {

    FJ_UNUSED (forcePolling);
    SimMag_t* pSim = (SimMag_t*)ctx;
    if (!pSim || !pField) {
        return eSTATUS_FAILURE;
    }

    /* Fixed field pointing north (normalised). */
    pField->x = 1.0F;
    pField->y = 0.0F;
    pField->z = 0.0F;
    return eSTATUS_SUCCESS;
}

STATIC bool Sim_IsDataReady (void* ctx) {
    SimMag_t* pSim = (SimMag_t*)ctx;
    if (!pSim) {
        return false;
    }
    return true;
}

eSTATUS_t MagDrv_Init (MagDriverConf_t const* pConf, MagDriver_t* pOutDriver) {

    if (!pConf || !pOutDriver) {
        return eSTATUS_NULL_ARG;
    }

    memset (pOutDriver, 0, sizeof (MagDriver_t));
    eSTATUS_t status = eSTATUS_SUCCESS;
    do {
        pOutDriver->ctx = Allocate (sizeof (SimMag_t));
        if (!pOutDriver->ctx) {
            status = eSTATUS_FAILURE;
            break;
        }

        pOutDriver->Read        = Sim_Read;
        pOutDriver->IsDataReady = Sim_IsDataReady;

    } while (0);

    return status;
}
