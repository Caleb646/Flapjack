#include <stdbool.h>
#include <stdint.h>

#include "common.h"

#include "cfg/cfg.h"

#include "core/core.h"

#include "drivers/bus/bus.h"
#include "drivers/bus/bus_defs.h"

#include "drivers/sensors/sensor.h"

extern eSTATUS_t Bmi323_InitAcc (AccCfg_t* pAccCfg, AccDevice_t* pOutAccDevice);
extern eSTATUS_t Bmi323_InitGyro (GyroCfg_t* pGyroCfg, GyroDevice_t* pOutGyroDevice);
extern eSTATUS_t Mmc5983_InitMag (MagCfg_t* pMagCfg, MagDevice_t* pOutMagDevice);

CFG_DEFINE (AccCfg_t, AccCfg);
CFG_DEFINE (GyroCfg_t, GyroCfg);
CFG_DEFINE (MagCfg_t, MagCfg);

FJ_DEFINE_SHARED (Acc_t, e_Acc);
FJ_DEFINE_SHARED (Gyro_t, e_Gyro);
FJ_DEFINE_SHARED (Mag_t, e_Mag);


FJ_STATIC FJ_INLINE bool IsMARG (AccCfg_t* pAccCfg, GyroCfg_t* pGyroCfg, MagCfg_t* pMagCfg) {
    return (pAccCfg->type == pGyroCfg->type) && (pAccCfg->type == pMagCfg->type);
}

FJ_STATIC FJ_INLINE bool IsIMU (AccCfg_t* pAccCfg, GyroCfg_t* pGyroCfg) {
    return (pAccCfg->type == pGyroCfg->type);
}

FJ_STATIC eSTATUS_t Sensor_InitBus (
AccCfg_t* pAccCfg,
AccDevice_t* pOutAccDevice,
GyroCfg_t* pGyroCfg,
GyroDevice_t* pOutGyroDevice,
MagCfg_t* pMagCfg,
MagDevice_t* pOutMagDevice
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

FJ_STATIC eSTATUS_t Sensor_InitHw (SENSOR_ID_t id, SENSOR_TYPE_t type, void* pCfg, void* pOutDevice) {

    switch (id) {
    case SENSOR_ID_BMI323:
        switch (type) {
        case SENSOR_TYPE_ACC: return Bmi323_InitAcc ((AccCfg_t*)pCfg, (AccDevice_t*)pOutDevice);
        case SENSOR_TYPE_GYR: return Bmi323_InitGyro ((GyroCfg_t*)pCfg, (GyroDevice_t*)pOutDevice);
        default: return eSTATUS_FAILURE;
        }
    case SENSOR_ID_MMC5983:
        switch (type) {
        case SENSOR_TYPE_MAG: return Mmc5983_InitMag ((MagCfg_t*)pCfg, (MagDevice_t*)pOutDevice);
        default: return eSTATUS_FAILURE;
        }
    default: return eSTATUS_FAILURE;
    }

    return eSTATUS_SUCCESS;
}

eSTATUS_t Sensors_Init (void) {

    AccCfg_t* pAccCfg   = AccCfg_GetMutable ();
    GyroCfg_t* pGyroCfg = GyroCfg_GetMutable ();
    MagCfg_t* pMagCfg   = MagCfg_GetMutable ();

    eSTATUS_t status = eSTATUS_SUCCESS;
    status = Sensor_InitBus (pAccCfg, &e_Acc.dev, pGyroCfg, &e_Gyro.dev, pMagCfg, &e_Mag.dev);
    GOTO_IF (FJ_FAIL (status), error, "Failed to init inertials base");

    status = Sensor_InitHw (pAccCfg->id, SENSOR_TYPE_ACC, pAccCfg, &e_Acc.dev);
    GOTO_IF (FJ_FAIL (status), error, "Failed to init ACC interface");

    status = Sensor_InitHw (pGyroCfg->id, SENSOR_TYPE_GYR, pGyroCfg, &e_Gyro.dev);
    GOTO_IF (FJ_FAIL (status), error, "Failed to init GYRO interface");

    if (TARG_MAX_MAGS > 0) {
        status = Sensor_InitHw (pMagCfg->id, SENSOR_TYPE_MAG, pMagCfg, &e_Mag.dev);
        GOTO_IF (FJ_FAIL (status), error, "Failed to init MAG interface");
    }

error:
    return status;
}

FJ_TESTABLE FJ_INLINE eSTATUS_t Acc_Update_ (Acc_t* pAcc, bool forcePolling) {

    if (!pAcc || !pAcc->dev.fnRead) {
        return eSTATUS_NULL_ARG;
    }

    eSTATUS_t status = pAcc->dev.fnRead (&pAcc->dev, forcePolling, &pAcc->rawData[0]);
    RETURN_IF (FJ_FAIL (status), status, "Failed to read accelerometer data");
    pAcc->scaledData.x = (float)pAcc->rawData[0] * pAcc->dev.scaleFactor;
    pAcc->scaledData.y = (float)pAcc->rawData[1] * pAcc->dev.scaleFactor;
    pAcc->scaledData.z = (float)pAcc->rawData[2] * pAcc->dev.scaleFactor;

    pAcc->filteredData = pAcc->scaledData; // No filtering yet

    return eSTATUS_SUCCESS;
}

eSTATUS_t Acc_Update (bool forcePolling) {
    return Acc_Update_ (&e_Acc, forcePolling);
}

eSTATUS_t Acc_Filter (void) {
}

bool Acc_IsAvailable (void) {
    return (e_Acc.dev.pBusDevice != NULL && e_Acc.dev.fnRead != NULL);
}

eSTATUS_t Gyro_Update (bool forcePolling) {
}

eSTATUS_t Gyro_Filter (void) {
}

bool Gyro_IsAvailable (void) {
}

eSTATUS_t Mag_Update (bool forcePolling) {
}

eSTATUS_t Mag_Filter (void) {
}

bool Mag_IsAvailable (void) {
}
