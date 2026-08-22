/*
 * Simulation IMU backend: blocks until the JSBSim bridge delivers the next
 * SensorData over the sim link, then returns its accel (m/s^2, specific force)
 * and gyro (deg/s) in the body FRD frame. Blocking here paces the IMU task -
 * and therefore the whole GNC chain - at the PC's sensor stream rate.
 */

#include "drivers/imu/imudrv.h"

#include "core/core.h"

#include "drivers/sim_link/sim_link.h"

#include "FreeRTOS.h"

#include <string.h>
#include <stdbool.h>

STATIC eSTATUS_t Sim_Read (void* ctx, bool forcePolling, ImuData_t* pOutData) {

    FJ_UNUSED (forcePolling);
    FJ_UNUSED (ctx);
    if (!pOutData) {
        return eSTATUS_FAILURE;
    }

    float accel[3];
    float gyro[3];
    float mag[3];
    if (!SimLink_WaitImu (accel, gyro, mag, portMAX_DELAY)) {
        return eSTATUS_FAILURE;
    }

    pOutData->accel.x = accel[0];
    pOutData->accel.y = accel[1];
    pOutData->accel.z = accel[2];
    pOutData->gyro.x  = gyro[0];
    pOutData->gyro.y  = gyro[1];
    pOutData->gyro.z  = gyro[2];
    return eSTATUS_SUCCESS;
}

STATIC bool Sim_IsDataReady (void* ctx) {
    FJ_UNUSED (ctx);
    return true;
}

eSTATUS_t ImuDrv_Init (ImuDriverConf_t const* pConf, ImuDriver_t* pOutDriver) {

    if (!pConf || !pOutDriver) {
        return eSTATUS_NULL_ARG;
    }

    memset (pOutDriver, 0, sizeof (ImuDriver_t));
    pOutDriver->ctx         = NULL;
    pOutDriver->Read        = Sim_Read;
    pOutDriver->IsDataReady = Sim_IsDataReady;
    return eSTATUS_SUCCESS;
}
