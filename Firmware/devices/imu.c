#include "devices/imu.h"

#include "drivers/imu/imudrv.h"

#include "target.h"

#include <string.h>

eSTATUS_t Imu_Init (Imu_t* pOutSensor, DataReadySignal_t const* pSignal) {

    if (!pOutSensor) {
        return eSTATUS_NULL_ARG;
    }

    memset (pOutSensor, 0, sizeof (Imu_t));

    pOutSensor->align = Align_Compose (IMU_ALIGN, CFG_BOARD_ALIGN);

    pOutSensor->drv.cfg.accRange  = eIMU_ACC_RANGE_2G;
    pOutSensor->drv.cfg.gyroRange = eIMU_GYRO_RANGE_250;
    pOutSensor->drv.cfg.odr       = eIMU_ODR_400;
    if (pSignal) {
        pOutSensor->drv.cfg.signal = *pSignal;
    }

    eSTATUS_t status = eSTATUS_SUCCESS;
    do {

        status = ImuDrv_Init (&pOutSensor->drv);

    } while(0);
    // TODO
    return status;
}

eSTATUS_t Imu_Update (Imu_t* pSensor) {

    if(!pSensor) {
        return eSTATUS_NULL_ARG;
    }

    eSTATUS_t status = pSensor->drv.Read (pSensor->drv.ctx, false, &pSensor->data);
    if (STATUS_FAIL (status)) {
        return status;
    }

    /*
     * Backends report specific force as the part measures it (a - g), so a
     * level sensor with its z axis pointing down reads -9.81 there. This
     * project's convention is the opposite sign - level and still is
     * (0, 0, +9.81) in FRD - see msgs/proto/defs/sim.proto and the Madgwick
     * filter's own comment in common/filter.c. Hence the whole-vector negation.
     *
     * ACCEL ONLY. This is a reflection (det = -1), not a rotation, and pushing
     * a reflection through a gyro inverts the sense of every rotation about a
     * mirrored axis - see KnownIssues 2.2 for what that costs. That is also why
     * it cannot be folded into `align`: eSensorAlign_t can only express proper
     * rotations, which makes the mistake unrepresentable rather than merely
     * discouraged.
     */
    pSensor->data.accel.x = -pSensor->data.accel.x;
    pSensor->data.accel.y = -pSensor->data.accel.y;
    pSensor->data.accel.z = -pSensor->data.accel.z;

    // Die frame -> body FRD. Same rotation for both, applied in place.
    Align_Apply (pSensor->align, &pSensor->data.accel, &pSensor->data.accel);
    Align_Apply (pSensor->align, &pSensor->data.gyro, &pSensor->data.gyro);

    // TODO: apply low-pass filtering (common/filter.h). Pass-through for now.
    pSensor->accelFiltered = pSensor->data.accel;
    pSensor->gyroFiltered  = pSensor->data.gyro;
    pSensor->usLastUpdate  = GetMicroseconds ();

    return eSTATUS_SUCCESS;
}
