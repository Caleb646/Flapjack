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

STATIC eSTATUS_t Sim_Read (void* ctx, bool forcePolling, Vec3f* pAccel, Vec3f* pGyro) {

    FJ_UNUSED (forcePolling);
    FJ_UNUSED (ctx);
    if (!pAccel || !pGyro) {
        return eSTATUS_FAILURE;
    }

    float accel[3];
    float gyro[3];
    float mag[3];
    if (!SimLink_WaitImu (accel, gyro, mag, portMAX_DELAY)) {
        return eSTATUS_FAILURE;
    }

    pAccel->x = accel[0];
    pAccel->y = accel[1];
    pAccel->z = accel[2];
    pGyro->x  = gyro[0];
    pGyro->y  = gyro[1];
    pGyro->z  = gyro[2];
    return eSTATUS_SUCCESS;
}

STATIC eSTATUS_t Sim_IsDataReady (void* ctx) {
    FJ_UNUSED (ctx);
    return eSTATUS_SUCCESS;
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
