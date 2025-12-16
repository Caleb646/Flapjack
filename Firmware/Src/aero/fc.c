#include <stdbool.h>
#include <stdint.h>

#include "common.h"

#include "core/core.h"

#include "aero/fc.h"

#include "drivers/motor.h"

#include "drivers/sensors/inertial/inertial.h"

#include "cfg/motor.h"

FJ_DEFINE_SHARED (Acc_t, e_Acc);


FJ_TESTABLE FJ_INLINE eSTATUS_t
Acc_Update_ (AccDevice_t* pAccDevice, float dt, bool forcePolling, int16_t* pOutRawData, Vec3f* pOutAcc) {

    eSTATUS_t status = pAccDevice->vtbl.fnAccReadData (pAccDevice, forcePolling, pOutRawData);
    RETURN_IF (FJ_FAIL (status), status, "Failed to read accelerometer data");
    pOutAcc->x = (float)pOutRawData[0] * pAccDevice->scaleFactor;
    pOutAcc->y = (float)pOutRawData[1] * pAccDevice->scaleFactor;
    pOutAcc->z = (float)pOutRawData[2] * pAccDevice->scaleFactor;
    return eSTATUS_SUCCESS;
}

eSTATUS_t Acc_Update (float dt, bool forcePolling) {

    if (!Acc_IsAvailable ()) {
        return eSTATUS_NOT_FOUND;
    }
    return Acc_Update_ (&e_Acc.dev, dt, forcePolling, &e_Acc.rawData[0], &e_Acc.scaledData);
}

eSTATUS_t Acc_Filter (float dt) {

    return eSTATUS_NOT_SUPPORTED;
}

bool Acc_IsAvailable (void) {
    AccDevice_t* pAccDevice = AccDevice_GetMutable ();
    return (pAccDevice != NULL) && (pAccDevice->pBusDevice != NULL);
}

FJ_TESTABLE FJ_INLINE eSTATUS_t Gyro_Update_ (GyroDevice_t* pGyroDevice, float dt, bool forcePolling, Vec3f* pOutGyro) {

    int16_t rawData[3] = { 0 };
    eSTATUS_t status   = pGyroDevice->vtbl.fnGyroReadData (pGyroDevice, forcePolling, rawData);
    RETURN_IF (FJ_FAIL (status), status, "Failed to read gyroscope data");
    pOutGyro->x = (float)rawData[0] * pGyroDevice->scaleFactor;
    pOutGyro->y = (float)rawData[1] * pGyroDevice->scaleFactor;
    pOutGyro->z = (float)rawData[2] * pGyroDevice->scaleFactor;
    return eSTATUS_SUCCESS;
}

eSTATUS_t Gyro_Update (float dt, bool forcePolling, Vec3f* pOutGyro) {

    GyroDevice_t* pGyroDevice = GyroDevice_GetMutable ();
    if (!pGyroDevice || !pGyroDevice->vtbl.fnGyroReadData || !pOutGyro) {
        return eSTATUS_NULL_ARG;
    }
    return Gyro_Update_ (pGyroDevice, dt, forcePolling, pOutGyro);
}

bool Gyro_IsAvailable (void) {
    GyroDevice_t* pGyroDevice = GyroDevice_GetMutable ();
    return (pGyroDevice != NULL) && (pGyroDevice->pBusDevice != NULL);
}

FJ_TESTABLE FJ_INLINE eSTATUS_t Mag_Update_ (MagDevice_t* pMagDevice, float dt, bool forcePolling, Vec3f* pOutMag) {

    int16_t rawData[3] = { 0 };
    eSTATUS_t status   = pMagDevice->vtbl.fnMagReadData (pMagDevice, forcePolling, rawData);
    RETURN_IF (FJ_FAIL (status), status, "Failed to read magnetometer data");
    // TODO: should update heading instead
    // pOutMag->x = (float)rawData[0] * pMagDevice->scaleFactor;
    // pOutMag->y = (float)rawData[1] * pMagDevice->scaleFactor;
    // pOutMag->z = (float)rawData[2] * pMagDevice->scaleFactor;
    return eSTATUS_SUCCESS;
}

eSTATUS_t Mag_Update (float dt, bool forcePolling, Vec3f* pOutMag) {

    MagDevice_t* pMagDevice = MagDevice_GetMutable ();
    if (!pMagDevice || !pMagDevice->vtbl.fnMagReadData || !pOutMag) {
        return eSTATUS_NULL_ARG;
    }
    return Mag_Update_ (pMagDevice, dt, forcePolling, pOutMag);
}

bool Mag_IsAvailable (void) {
    MagDevice_t* pMagDevice = MagDevice_GetMutable ();
    return (pMagDevice != NULL) && (pMagDevice->pBusDevice != NULL);
}