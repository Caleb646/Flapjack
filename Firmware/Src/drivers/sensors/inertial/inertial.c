#include <stdbool.h>
#include <stdint.h>

#include "common.h"

#include "core/core.h"

#include "drivers/driver.h"

#include "drivers/bus/bus.h"
#include "drivers/bus/bus_defs.h"

#include "drivers/sensors/inertial/inertial.h"

#include "cfg/sensors/sensor.h"

extern eSTATUS_t Bmi323_InitAcc (AccCfg_t* pAccCfg, ACCDevice_t* pOutAccDevice);
extern eSTATUS_t Bmi323_InitGyro (GyroCfg_t* pGyroCfg, GYRODevice_t* pOutGyroDevice);
extern eSTATUS_t Mmc5983_InitMag (MagCfg_t* pMagCfg, MAGDevice_t* pOutMagDevice);

DRIVER_DEFINE (ACCDevice_t, AccDevice);
DRIVER_DEFINE (GYRODevice_t, GyroDevice);
DRIVER_DEFINE (MAGDevice_t, MagDevice);

FJ_STATIC FJ_INLINE bool IsMARG (AccCfg_t* pAccCfg, GyroCfg_t* pGyroCfg, MagCfg_t* pMagCfg) {
    return (pAccCfg->type == pGyroCfg->type) && (pAccCfg->type == pMagCfg->type);
}

FJ_STATIC FJ_INLINE bool IsIMU (AccCfg_t* pAccCfg, GyroCfg_t* pGyroCfg) {
    return (pAccCfg->type == pGyroCfg->type);
}

FJ_STATIC eSTATUS_t Inertial_BaseInit (
AccCfg_t* pAccCfg,
ACCDevice_t* pOutAccDevice,
GyroCfg_t* pGyroCfg,
GYRODevice_t* pOutGyroDevice,
MagCfg_t* pMagCfg,
MAGDevice_t* pOutMagDevice
) {

    eSTATUS_t status = eSTATUS_SUCCESS;

    if (IsMARG (pAccCfg, pGyroCfg, pMagCfg)) {

        BusDevice_t* pBusDevice = (BusDevice_t*)Alloc_SharedMem (sizeof (BusDevice_t));
        status                  = Bus_Init (&pAccCfg->busCfg, pBusDevice);
        GOTO_IF (FJ_FAIL (status), error, "Failed to init bus device for MARG");

        pOutAccDevice->pBusDevice  = pBusDevice;
        pOutGyroDevice->pBusDevice = pBusDevice;
        pOutMagDevice->pBusDevice  = pBusDevice;

    } else if (IsIMU (pAccCfg, pGyroCfg)) {

        BusDevice_t* pBusDevice = (BusDevice_t*)Alloc_SharedMem (sizeof (BusDevice_t));
        status                  = Bus_Init (&pAccCfg->busCfg, pBusDevice);
        GOTO_IF (FJ_FAIL (status), error, "Failed to init bus device for IMU");

        pOutAccDevice->pBusDevice  = pBusDevice;
        pOutGyroDevice->pBusDevice = pBusDevice;

        if (TARG_MAX_MAGS > 0) {
            BusDevice_t* pMagBusDevice = (BusDevice_t*)Alloc_SharedMem (sizeof (BusDevice_t));
            status                     = Bus_Init (&pMagCfg->busCfg, pMagBusDevice);
            GOTO_IF (FJ_FAIL (status), error, "Failed to init bus device for MAG");
            pOutMagDevice->pBusDevice = pMagBusDevice;
        }

    } else {
        BusDevice_t* pAccBusDevice = (BusDevice_t*)Alloc_SharedMem (sizeof (BusDevice_t));
        status                     = Bus_Init (&pAccCfg->busCfg, pAccBusDevice);
        GOTO_IF (FJ_FAIL (status), error, "Failed to init bus device for ACC");
        pOutAccDevice->pBusDevice = pAccBusDevice;

        BusDevice_t* pGyroBusDevice = (BusDevice_t*)Alloc_SharedMem (sizeof (BusDevice_t));
        status                      = Bus_Init (&pGyroCfg->busCfg, pGyroBusDevice);
        GOTO_IF (FJ_FAIL (status), error, "Failed to init bus device for GYRO");
        pOutGyroDevice->pBusDevice = pGyroBusDevice;

        if (TARG_MAX_MAGS > 0) {
            BusDevice_t* pMagBusDevice = (BusDevice_t*)Alloc_SharedMem (sizeof (BusDevice_t));
            status                     = Bus_Init (&pMagCfg->busCfg, pMagBusDevice);
            GOTO_IF (FJ_FAIL (status), error, "Failed to init bus device for MAG");
            pOutMagDevice->pBusDevice = pMagBusDevice;
        }
    }

error:
    return status;
}

FJ_STATIC eSTATUS_t Inertial_InterfaceInit (INER_INTERFACE_ID_t id, INER_TYPE_t type, void* pCfg, void* pOutDevice) {

    switch (id) {
    case INER_INTERFACE_ID_BMI323:
        switch (type) {
        case INER_TYPE_ACC: return Bmi323_InitAcc ((AccCfg_t*)pCfg, (ACCDevice_t*)pOutDevice);
        case INER_TYPE_GYR: return Bmi323_InitGyro ((GyroCfg_t*)pCfg, (GYRODevice_t*)pOutDevice);
        default: return eSTATUS_FAILURE;
        }
    case INER_INTERFACE_ID_MMC5983:
        switch (type) {
        case INER_TYPE_MAG: return Mmc5983_InitMag ((MagCfg_t*)pCfg, (MAGDevice_t*)pOutDevice);
        default: return eSTATUS_FAILURE;
        }
    default: return eSTATUS_FAILURE;
    }

    return eSTATUS_SUCCESS;
}

eSTATUS_t Inertial_Init (void) {

    AccCfg_t* pAccCfg   = AccCfg_GetMutable ();
    GyroCfg_t* pGyroCfg = GyroCfg_GetMutable ();
    MagCfg_t* pMagCfg   = MagCfg_GetMutable ();

    ACCDevice_t* pAccDevice   = AccDevice_GetMutable ();
    GYRODevice_t* pGyroDevice = GyroDevice_GetMutable ();
    MAGDevice_t* pMagDevice   = MagDevice_GetMutable ();

    eSTATUS_t status = eSTATUS_SUCCESS;
    status = Inertial_BaseInit (pAccCfg, pAccDevice, pGyroCfg, pGyroDevice, pMagCfg, pMagDevice);
    GOTO_IF (FJ_FAIL (status), error, "Failed to init inertials base");

    status = Inertial_InterfaceInit (pAccCfg->type, INER_TYPE_ACC, pAccCfg, pAccDevice);
    GOTO_IF (FJ_FAIL (status), error, "Failed to init ACC interface");

    status = Inertial_InterfaceInit (pGyroCfg->type, INER_TYPE_GYR, pGyroCfg, pGyroDevice);
    GOTO_IF (FJ_FAIL (status), error, "Failed to init GYRO interface");

    if (TARG_MAX_MAGS > 0) {
        status = Inertial_InterfaceInit (pMagCfg->type, INER_TYPE_MAG, pMagCfg, pMagDevice);
        GOTO_IF (FJ_FAIL (status), error, "Failed to init MAG interface");
    }

error:
    return status;
}

eSTATUS_t Acc_Update (float dt, bool forcePolling, Vec3f* pOutAcc) {
}

eSTATUS_t Gyro_Update (float dt, bool forcePolling, Vec3f* pOutGyro) {
}

eSTATUS_t Mag_Update (float dt, bool forcePolling, Vec3f* pOutMag) {
}
