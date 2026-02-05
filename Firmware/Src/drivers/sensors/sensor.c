#include <stdbool.h>
#include <stdint.h>

#include "common.h"

#include "cfg/cfg.h"

#include "drivers/core/log.h"

#include "drivers/bus/bus.h"
#include "drivers/bus/bus_defs.h"

#include "drivers/sensors/sensor.h"

#include "platform/platform.h"

extern eSTATUS_t Bmi323_InitAcc (AccCfg_t* pAccCfg, AccDevice_t* pOutAccDevice);
extern eSTATUS_t Bmi323_InitGyro (GyroCfg_t* pGyroCfg, GyroDevice_t* pOutGyroDevice);
extern eSTATUS_t Mmc5983_InitMag (MagCfg_t* pMagCfg, MagDevice_t* pOutMagDevice);

// CFG_DEFINE (AccCfg_t, AccCfg);
// CFG_DEFINE (GyroCfg_t, GyroCfg);
// CFG_DEFINE (MagCfg_t, MagCfg);

FJ_DEFINE_SHARED (Acc_t, e_Acc);
FJ_DEFINE_SHARED (Gyro_t, e_Gyro);
FJ_DEFINE_SHARED (Mag_t, e_Mag);


// FJ_STATIC eSTATUS_t Sensor_InitHw (SENSOR_ID_t id, SENSOR_TYPE_t type, void* pCfg, void* pOutDevice) {

//     switch (id) {
//     case SENSOR_ID_BMI323:
//         switch (type) {
//         case SENSOR_TYPE_ACC: return Bmi323_InitAcc ((AccCfg_t*)pCfg, (AccDevice_t*)pOutDevice);
//         case SENSOR_TYPE_GYR: return Bmi323_InitGyro ((GyroCfg_t*)pCfg,
//         (GyroDevice_t*)pOutDevice); default: return eSTATUS_FAIL;
//         }
//     case SENSOR_ID_MMC5983:
//         switch (type) {
//         case SENSOR_TYPE_MAG: return Mmc5983_InitMag ((MagCfg_t*)pCfg, (MagDevice_t*)pOutDevice);
//         default: return eSTATUS_FAIL;
//         }
//     default: return eSTATUS_FAIL;
//     }

//     return eSTATUS_OK;
// }

/*
 * Add initialization for gyro, mag, gps, baro......
 */
eSTATUS_t Acc_Init (void) {

    Acc_t* pAcc     = &e_Acc;
    AccCfg_t accCfg = { 0 };
    accCfg.iface    = SENSOR_IFACE_MAKE (TARG_ACC);

#if TARG_ACC_SPI_ENABLED()
    accCfg.busCfg.busType      = eBUS_TYPE_SPI;
    accCfg.busCfg.busId        = BUS_ID_MAKE (TARG_ACC_SPI);
    accCfg.busCfg.spiNssGpioId = GPIO_ID_MAKE (TARG_ACC_SPI_NSS);
#endif

    BusDevice_t* pBusDevice = Bus_Init (&accCfg.busCfg);
    if (!pBusDevice) {
        return eSTATUS_FAIL;
    }
    pAcc->dev.pBusDevice = pBusDevice;

    switch (accCfg.iface) {
    case eSENSOR_IFACE_BMI323: return Bmi323_InitAcc (&accCfg, &pAcc->dev); break;
    default: return eSTATUS_FAIL;
    }

    return eSTATUS_OK;
}

FJ_TESTABLE FJ_INLINE eSTATUS_t Acc_Update_ (Acc_t* pAcc, bool forcePolling) {

    if (!pAcc || !pAcc->dev.fnRead) {
        return eSTATUS_OK;
    }

    eSTATUS_t status = pAcc->dev.fnRead (&pAcc->dev, forcePolling, &pAcc->rawData[0]);
    RETURN_IF (FJ_FAIL (status), status, "Failed to read accelerometer data");
    pAcc->scaledData.x = (float)pAcc->rawData[0] * pAcc->dev.scaleFactor;
    pAcc->scaledData.y = (float)pAcc->rawData[1] * pAcc->dev.scaleFactor;
    pAcc->scaledData.z = (float)pAcc->rawData[2] * pAcc->dev.scaleFactor;
    // TODO filter
    pAcc->filteredData = pAcc->scaledData; // No filtering yet

    return eSTATUS_OK;
}

eSTATUS_t Acc_Update (uint32_t currentTimeUs, bool forcePolling) {
    return Acc_Update_ (&e_Acc, forcePolling);
}

eSTATUS_t Acc_Filter (void) {
}

bool Acc_IsAvailable (void) {
    return (e_Acc.dev.pBusDevice != NULL && e_Acc.dev.fnRead != NULL);
}

eSTATUS_t Gyro_Update (uint32_t currentTimeUs, bool forcePolling) {
}

eSTATUS_t Gyro_Filter (void) {
}

bool Gyro_IsAvailable (void) {
}

eSTATUS_t Mag_Update (uint32_t currentTimeUs, bool forcePolling) {
}

eSTATUS_t Mag_Filter (void) {
}

bool Mag_IsAvailable (void) {
}
