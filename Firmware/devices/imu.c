#include "devices/imu.h"

#include "drivers/imu/imudrv.h"

#include <string.h>

eSTATUS_t Imu_Init (Imu_t* pOutSensor) {

    if (!pOutSensor) {
        return eSTATUS_NULL_ARG;
    }

    memset (pOutSensor, 0, sizeof (Imu_t));

    ImuDriverConf_t conf = {
        .accRange    = eIMU_ACC_RANGE_2G,
        .gyroRange   = eIMU_GYRO_RANGE_250,
        .odr         = eIMU_ODR_400,
        .orientation = {
            .remap = eIMU_AXES_REMAP_YXZ,
            .xDir  = eIMU_AXES_DIR_INVERTED,
            .yDir  = eIMU_AXES_DIR_INVERTED,
            .zDir  = eIMU_AXES_DIR_INVERTED,
        },
    };

    eSTATUS_t status = eSTATUS_SUCCESS;
    do {

        status = ImuDrv_Init(&conf, &pOutSensor->drv);

    } while(0);
    // TODO
    return status;
}

eSTATUS_t Imu_Update (Imu_t* pSensor) {

    if(!pSensor) {
        return eSTATUS_NULL_ARG;
    }

    eSTATUS_t status = pSensor->drv.Read (pSensor->drv.ctx, false, &pSensor->accel, &pSensor->gyro);
    if (STATUS_FAIL (status)) {
        return status;
    }

    // TODO: apply low-pass filtering (common/filter.h). Pass-through for now.
    pSensor->accelFiltered = pSensor->accel;
    pSensor->gyroFiltered  = pSensor->gyro;
    pSensor->usLastUpdate  = GetMicroseconds ();

    return eSTATUS_SUCCESS;
}
