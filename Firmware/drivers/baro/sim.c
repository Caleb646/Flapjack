/*
 * Simulation barometer backend: blocks until the JSBSim bridge delivers the next
 * BaroData over the sim link, then returns its static pressure (Pa) and
 * temperature (degrees C). Blocking here paces Baro_Task at the PC's baro stream
 * rate the same way the sim IMU backend paces Imu_Task - and because BaroData is
 * its own frame at its own rate, that is ~50 Hz rather than the 400 Hz of the
 * SensorData stream.
 */

#include "drivers/baro/barodrv.h"

#include "core/core.h"

#include "drivers/sim_link/sim_link.h"

#include "FreeRTOS.h"

#include <string.h>
#include <stdbool.h>

STATIC eSTATUS_t Sim_Read (void* ctx, bool forcePolling, BaroData_t* pOutData) {

    FJ_UNUSED (forcePolling);
    FJ_UNUSED (ctx);
    if (!pOutData) {
        return eSTATUS_NULL_ARG;
    }

    if (!SimLink_WaitBaro (&pOutData->pressurePa, &pOutData->temperatureC, portMAX_DELAY)) {
        return eSTATUS_FAILURE;
    }
    return eSTATUS_SUCCESS;
}

STATIC bool Sim_IsDataReady (void* ctx) {
    FJ_UNUSED (ctx);
    return true;
}

eSTATUS_t BaroDrv_Init (BaroDriverConf_t const* pConf, BaroDriver_t* pOutDriver) {

    if (!pConf || !pOutDriver) {
        return eSTATUS_NULL_ARG;
    }

    memset (pOutDriver, 0, sizeof (BaroDriver_t));
    pOutDriver->ctx         = NULL;
    pOutDriver->Read        = Sim_Read;
    pOutDriver->IsDataReady = Sim_IsDataReady;
    return eSTATUS_SUCCESS;
}
