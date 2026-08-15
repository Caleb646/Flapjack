/*
 * Simulation magnetometer backend: blocks until the JSBSim bridge delivers the
 * next SensorData, then returns the normalized field it synthesized from
 * vehicle attitude. Blocking here paces Mag_Task at the PC's sensor stream rate,
 * the same way the sim IMU backend paces Imu_Task.
 */

#include "drivers/mag/magdrv.h"

#include "core/core.h"

#include "drivers/sim_link/sim_link.h"

#include "FreeRTOS.h"

#include <string.h>
#include <stdbool.h>

STATIC eSTATUS_t Sim_Read (void* ctx, bool forcePolling, Vec3f* pField) {

    FJ_UNUSED (forcePolling);
    FJ_UNUSED (ctx);
    if (!pField) {
        return eSTATUS_FAILURE;
    }

    float mag[3];
    if (!SimLink_WaitMag (mag, portMAX_DELAY)) {
        return eSTATUS_FAILURE;
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
