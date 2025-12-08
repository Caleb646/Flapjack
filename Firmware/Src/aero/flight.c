#include <stdbool.h>
#include <stdint.h>

#include "common.h"

#include "core/core.h"

#include "aero/flight.h"

#include "drivers/motor.h"

#include "drivers/sensors/inertial/inertial.h"

#include "cfg/motor.h"


ACCDevice_t* Flight_GetActive_AccDevice (void) {
    return AccDevice_GetMutable ();
}

GYRODevice_t* Flight_GetActive_GyroDevice (void) {
    return GyroDevice_GetMutable ();
}

MAGDevice_t* Flight_GetActive_MagDevice (void) {
    return MagDevice_GetMutable ();
}

bool Flight_HasAcc (void) {
    ACCDevice_t* pAccDevice = Flight_GetActive_AccDevice ();
    return (pAccDevice != NULL) && (pAccDevice->pBusDevice != NULL);
}

bool Flight_HasGyro (void) {
    GYRODevice_t* pGyroDevice = Flight_GetActive_GyroDevice ();
    return (pGyroDevice != NULL) && (pGyroDevice->pBusDevice != NULL);
}

bool Flight_HasMag (void) {
    MAGDevice_t* pMagDevice = Flight_GetActive_MagDevice ();
    return (pMagDevice != NULL) && (pMagDevice->pBusDevice != NULL);
}

eSTATUS_t Acc_Update (ACCDevice_t* pAccDevice, float dt, bool forcePolling, Vec3f* pOutAcc) {

    if (!pAccDevice || !pAccDevice->vtbl.fnAccReadData || !pOutAcc) {
        return eSTATUS_NULL_ARG;
    }

    int16_t rawData[3] = { 0 };
    eSTATUS_t status   = pAccDevice->vtbl.fnAccReadData (pAccDevice, forcePolling, rawData);
    RETURN_IF (FJ_FAIL (status), status, "Failed to read accelerometer data");
    pOutAcc->x = (float)rawData[0] * pAccDevice->scaleFactor;
    pOutAcc->y = (float)rawData[1] * pAccDevice->scaleFactor;
    pOutAcc->z = (float)rawData[2] * pAccDevice->scaleFactor;
    return eSTATUS_SUCCESS;
}

eSTATUS_t Gyro_Update (GYRODevice_t* pGyroDevice, float dt, bool forcePolling, Vec3f* pOutGyro) {

    if (!pGyroDevice || !pGyroDevice->vtbl.fnGyroReadData || !pOutGyro) {
        return eSTATUS_NULL_ARG;
    }

    int16_t rawData[3] = { 0 };
    eSTATUS_t status   = pGyroDevice->vtbl.fnGyroReadData (pGyroDevice, forcePolling, rawData);
    RETURN_IF (FJ_FAIL (status), status, "Failed to read gyroscope data");
    pOutGyro->x = (float)rawData[0] * pGyroDevice->scaleFactor;
    pOutGyro->y = (float)rawData[1] * pGyroDevice->scaleFactor;
    pOutGyro->z = (float)rawData[2] * pGyroDevice->scaleFactor;
    return eSTATUS_SUCCESS;
}

eSTATUS_t Mag_Update (MAGDevice_t* pMagDevice, float dt, bool forcePolling, Vec3f* pOutMag) {

    if (!pMagDevice || !pMagDevice->vtbl.fnMagReadData || !pOutMag) {
        return eSTATUS_NULL_ARG;
    }

    int16_t rawData[3] = { 0 };
    eSTATUS_t status   = pMagDevice->vtbl.fnMagReadData (pMagDevice, forcePolling, rawData);
    RETURN_IF (FJ_FAIL (status), status, "Failed to read magnetometer data");
    // TODO
    // pOutMag->x = (float)rawData[0] * pMagDevice->scaleFactor;
    // pOutMag->y = (float)rawData[1] * pMagDevice->scaleFactor;
    // pOutMag->z = (float)rawData[2] * pMagDevice->scaleFactor;
    return eSTATUS_NOT_SUPPORTED;
    // return eSTATUS_SUCCESS;
}

eSTATUS_t Init_Motion (void) {

    eSTATUS_t status = eSTATUS_SUCCESS;

    for (uint32_t i = 0; i < MotionCfgs_GetSize (); ++i) {

        MotorCfg_t* pMotorCfg = MotorCfgs_GetMutable (i);
        MotorDevice_t* pMotor = MotorDevices_GetMutable (i);
        status                = Motor_Init (pMotorCfg, pMotor);
        RETURN_IF (FJ_FAIL (status), status, "Failed to initialize motor ID %d", pMotorCfg->id);
    }

    for (uint32_t i = 0; i < ServoCfgs_GetSize (); ++i) {

        ServoCfg_t* pServoCfg = ServoCfgs_GetMutable (i);
        ServoDevice_t* pServo = ServoDevices_GetMutable (i);
        status                = Servo_Init (pServoCfg, pServo);
        RETURN_IF (FJ_FAIL (status), status, "Failed to initialize servo ID %d", pServoCfg->id);
    }

    return eSTATUS_SUCCESS;
}

eSTATUS_t Motion_Update (Vec3f* const pidAtt, float targetThrottle, float dt) {
}