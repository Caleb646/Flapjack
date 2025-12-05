#include <stdbool.h>
#include <stdint.h>

#include "common.h"

#include "core/core.h"

#include "drivers/driver.h"

#include "drivers/bus/bus.h"
#include "drivers/bus/bus_defs.h"

#include "drivers/bus/spi.h"
#include "drivers/bus/spi_defs.h"

#include "drivers/sensors/inertial/bmixxx.h"
#include "drivers/sensors/inertial/inertial.h"

#include "cfg/sensors/sensor.h"

static bool g_isBmi323Initialized = false;

typedef struct BmiSensorConf_s {
    union {
        uint16_t raw;
        struct {
            uint16_t odr : 4;
            uint16_t range : 3;
            uint16_t bw : 1;
            uint16_t avg : 3;
            uint16_t res1 : 1;
            uint16_t mode : 3;
            uint16_t res2 : 1;
        };
    };
} BmiSensorConf_t;

typedef struct BmiSensorAltConf_s {
    union {
        uint16_t raw;
        struct {
            uint16_t odr : 4;
            uint16_t res1 : 4;
            uint16_t avg : 3;
            uint16_t res2 : 1;
            uint16_t mode : 3;
            uint16_t res3 : 1;
        };
    };
} BmiSensorAltConf_t;

typedef struct BmiFeat_s {
    union {
        uint16_t raw;
        struct {
            uint16_t err : 4;
            uint16_t scStComplete : 1;
            uint16_t gyroSelfCalibComplete : 1;
            uint16_t selfTestRes : 1;
            uint16_t sampleRateErr : 1;
            uint16_t res1 : 2;
            uint16_t axisRemapComplete : 1;
            uint16_t sysState : 2;
            uint16_t res2 : 3;
        };
    };
} BmiFeat_t;

typedef struct BmiIntStatus_s {
    union {
        uint16_t raw;
        struct {
            uint16_t unused : 10;
            uint16_t errStatus : 1;
            uint16_t drdyTemp : 1;
            uint16_t drdyGyro : 1;
            uint16_t drdyAccel : 1;
        };
    };
} BmiIntStatus_t;

FJ_STATIC eSTATUS_t Bmi323_SendCmd (BusDeviceSPI_t* pBusDeviceSpi, uint16_t cmd) {

    uint8_t pRegData[2] = { cmd & BMI3_SET_LOW_BYTE, (cmd & BMI3_SET_HIGH_BYTE) >> 8U };
    eSTATUS_t status    = SPI_WriteRegister (pBusDeviceSpi, BMI3_REG_CMD, pRegData, 2);
    RETURN_IF (FJ_FAIL (status), status, "Failed to send bmi323 command 0x%04X", cmd);
    return eSTATUS_SUCCESS;
}

FJ_STATIC eSTATUS_t Bmi323_SoftReset (BusDeviceSPI_t* pBusDeviceSpi) {

    /* Send soft reset command to BMI323 */
    eSTATUS_t status = Bmi323_SendCmd (pBusDeviceSpi, BMI3_CMD_SOFT_RESET);
    RETURN_IF (FJ_FAIL (status), status, "Failed to send soft reset command to bmi323");
    Delay (10);

    /* Perform dummy read to switch from I3C/I2C to SPI */
    uint8_t dummyBytes[2] = { 0 };
    status                = SPI_ReadRegister (pBusDeviceSpi, BMI3_REG_CHIP_ID, dummyBytes, 2);
    RETURN_IF (FJ_FAIL (status), status, "Failed to perform dummy read after soft reset");
    DelayMicroseconds (2);

    /* Enable feature engine */
    uint8_t featureData[2] = { 0x2C, 0x01 };
    status = SPI_WriteRegister (pBusDeviceSpi, BMI3_REG_FEATURE_IO2, featureData, 2);
    RETURN_IF (FJ_FAIL (status), status, "Failed to enable bmi323 feature engine");
    DelayMicroseconds (2);

    /* Enable feature status bit */
    uint8_t featureIOStatus[2] = { BMI3_ENABLE, 0 };
    status = SPI_WriteRegister (pBusDeviceSpi, BMI3_REG_FEATURE_IO_STATUS, featureIOStatus, 2);
    RETURN_IF (FJ_FAIL (status), status, "Failed to enable feature status bit");
    DelayMicroseconds (2);

    /* Enable feature engine bit */
    uint8_t featureEngine[2] = { BMI3_ENABLE, 0 };
    status = SPI_WriteRegister (pBusDeviceSpi, BMI3_REG_FEATURE_CTRL, featureEngine, 2);
    RETURN_IF (FJ_FAIL (status), status, "Failed to enable feature engine bit");
    DelayMicroseconds (2);

    uint8_t featEnabled = false;
    int16_t loop        = 0;
    uint8_t regData[2]  = { 0 };

    while (loop++ <= 20) {
        Delay (100);
        status = SPI_ReadRegister (pBusDeviceSpi, BMI3_REG_FEATURE_IO1, regData, 2);
        if (FJ_OK (status) && (regData[0] & (uint16_t)BMI3_FEATURE_ENGINE_ENABLE_MASK)) {
            featEnabled = true;
            break;
        }
    }

    if (!featEnabled) {
        LOG_ERROR ("Failed to enable feature engine after soft reset");
        return eSTATUS_FAILURE;
    }
    return status;
}

FJ_STATIC eSTATUS_t Bmi323_RemapAxes (BusDeviceSPI_t* pBusDeviceSpi, uint8_t remap) {

    uint8_t addr[2] = { BMI3_BASE_ADDR_AXIS_REMAP, 0 };
    /* Set the configuration to feature engine register */
    eSTATUS_t status = SPI_WriteRegister (pBusDeviceSpi, BMI3_REG_FEATURE_DATA_ADDR, addr, 2U);
    RETURN_IF (FJ_FAIL (status), status, "Failed to set bmi323 feature data address for axis remap");
    DelayMicroseconds (2);

    uint8_t aSend[2] = { remap, 0 };
    status           = SPI_WriteRegister (pBusDeviceSpi, BMI3_REG_FEATURE_DATA_TX, aSend, 2U);
    RETURN_IF (FJ_FAIL (status), status, "Failed to set bmi323 feature data TX for axis remap");
    DelayMicroseconds (2);

    /*
     * NOTE: The command to start the axis remap update can be sent without
     * checking the enabled/disabled status of the accel because this
     * function is only called after an bmi323 soft reset.
     */
    status = Bmi323_SendCmd (pBusDeviceSpi, BMI3_CMD_AXIS_MAP_UPDATE);
    RETURN_IF (FJ_FAIL (status), status, "Failed to send bmi323 command to update axis remap");

    int16_t wait = 1000;
    status       = eSTATUS_FAILURE;
    while (wait-- > 0) {

        BmiFeat_t featStatus = { 0 };
        status = SPI_ReadRegister (pBusDeviceSpi, BMI3_REG_FEATURE_IO1, (uint8_t*)&featStatus.raw, 2U);
        RETURN_IF (FJ_FAIL (status), status, "Failed to get bmi323 feature status");
        // clang-format off
        if (featStatus.err == eBMI3_FEAT_ERROR_NO_ERROR && 
            featStatus.sysState == eBMI3_FEAT_STATE_SYS_IN_FEAT_MODE && 
            featStatus.axisRemapComplete == eBMI3_FEAT_AXIS_MAP_COMPLETE) {
            LOG_INFO ("bmi323 axis remap successful");
            return eSTATUS_SUCCESS;
        }
        // clang-format on
        Delay (1);
    }
    return status;
}

FJ_STATIC eSTATUS_t Bmi323_Calibrate (BusDeviceSPI_t* pBusDeviceSpi) {

    BmiSensorConf_t accConf = { 0 };
    eSTATUS_t status = SPI_ReadRegister (pBusDeviceSpi, BMI3_REG_ACC_CONF, (uint8_t*)&accConf.raw, 2);
    RETURN_IF (FJ_FAIL (status), status, "Failed to read bmi323 alternate accelerometer config before calibration");
    DelayMicroseconds (2);

    BmiSensorConf_t gyroConf = { 0 };
    status = SPI_ReadRegister (pBusDeviceSpi, BMI3_REG_GYR_CONF, (uint8_t*)&gyroConf.raw, 2);
    RETURN_IF (FJ_FAIL (status), status, "Failed to read bmi323 gyroscope config before calibration");
    DelayMicroseconds (2);

    BmiSensorConf_t calibAccConf = { 0 };
    calibAccConf.raw             = accConf.raw;
    calibAccConf.mode            = BMI3_ACC_MODE_HIGH_PERF;
    calibAccConf.odr             = BMI3_ACC_ODR_100HZ;
    status = SPI_WriteRegister (pBusDeviceSpi, BMI3_REG_ACC_CONF, (uint8_t*)&calibAccConf.raw, 2);
    RETURN_IF (FJ_FAIL (status), status, "Failed to set bmi323 accelerometer config for calibration");
    DelayMicroseconds (2);

    BmiSensorAltConf_t altAccConf = { 0 };
    altAccConf.mode               = BMI3_ACC_MODE_DISABLE;
    status = SPI_ReadRegister (pBusDeviceSpi, BMI3_REG_ALT_ACC_CONF, (uint8_t*)&altAccConf.raw, 2);
    RETURN_IF (FJ_FAIL (status), status, "Failed to read bmi323 alternate accelerometer config before calibration");
    DelayMicroseconds (2);

    BmiSensorAltConf_t altGyroConf = { 0 };
    altGyroConf.mode               = BMI3_GYR_MODE_DISABLE;
    status = SPI_ReadRegister (pBusDeviceSpi, BMI3_REG_ALT_GYR_CONF, (uint8_t*)&altGyroConf.raw, 2);
    RETURN_IF (FJ_FAIL (status), status, "Failed to read bmi323 alternate gyroscope config before calibration");
    DelayMicroseconds (2);

    status = Bmi323_SendCmd (pBusDeviceSpi, BMI3_CMD_SELF_CALIB_TRIGGER);
    RETURN_IF (FJ_FAIL (status), status, "Failed to send bmi323 self-calibration trigger command");
    DelayMicroseconds (2);

    for (uint8_t idx = 0; idx < 10U; idx++) {
        /* A delay of 1000ms (100ms * 10(limit)) is required to perform self calibration */
        HAL_Delay (100);
        uint16_t intStatus = 0;
        status = SPI_ReadRegister (pBusDeviceSpi, BMI3_REG_INT_STATUS_INT1, (uint8_t*)&intStatus, 2);
        GOTO_IF (FJ_FAIL (status), error, "Failed to get bmi323 interrupt status during self-calibration");
        // calib is finished
        if (intStatus & BMI3_ERROR_STATUS_MASK) {
            break;
        }
    }

    BmiFeat_t featStatus = { 0 };
    status = SPI_ReadRegister (pBusDeviceSpi, BMI3_REG_FEATURE_IO1, (uint8_t*)&featStatus.raw, 2);
    GOTO_IF (FJ_FAIL (status), error, "Failed to get bmi323 feature status after self-calibration");
    // clang-format off
    if (featStatus.err != eBMI3_FEAT_ERROR_NO_ERROR || 
        featStatus.sysState != eBMI3_FEAT_STATE_SYS_IN_FEAT_MODE || 
        featStatus.scStComplete != eBMI3_FEAT_CALIB_COMPLETE) {
        LOG_ERROR ("bmi323 self-calibration failed");
        goto error;
    }
    // clang-format on

error:
    SPI_WriteRegister (pBusDeviceSpi, BMI3_REG_ACC_CONF, (uint8_t*)&accConf.raw, 2);
    DelayMicroseconds (2);
    SPI_WriteRegister (pBusDeviceSpi, BMI3_REG_GYR_CONF, (uint8_t*)&gyroConf.raw, 2);
    DelayMicroseconds (2);
    SPI_WriteRegister (pBusDeviceSpi, BMI3_REG_ALT_ACC_CONF, (uint8_t*)&altAccConf.raw, 2);
    DelayMicroseconds (2);
    SPI_WriteRegister (pBusDeviceSpi, BMI3_REG_ALT_GYR_CONF, (uint8_t*)&altGyroConf.raw, 2);
    DelayMicroseconds (2);
    return status;
}

FJ_STATIC bool Bmi323_IsAccDataReady (BusDeviceSPI_t* pBusDeviceSpi) {

    BmiIntStatus_t intStatus = { 0 };
    SPI_ReadRegister (pBusDeviceSpi, BMI3_REG_INT_STATUS_INT1, (uint8_t*)&(intStatus.raw), 2);
    return intStatus.drdyAccel != 0;
}

FJ_STATIC bool Bmi323_IsGyroDataReady (BusDeviceSPI_t* pBusDeviceSpi) {

    BmiIntStatus_t intStatus = { 0 };
    SPI_ReadRegister (pBusDeviceSpi, BMI3_REG_INT_STATUS_INT1, (uint8_t*)&(intStatus.raw), 2);
    return intStatus.drdyGyro != 0;
}


FJ_STATIC eSTATUS_t Bmi323_ReadAccData (ACCDevice_t* pAccDevice, bool forcePolling, int16_t* pOutData) {

    if (!pAccDevice || !pAccDevice->pBusDevice || !pOutData) {
        return eSTATUS_NULL_ARG;
    }

    uint16_t timeout = 10000U; // 50 ms timeout
    while (forcePolling && !Bmi323_IsAccDataReady (&(pAccDevice->pBusDevice->spi)) && timeout-- > 0) {
        DelayMicroseconds (5);
    }

    uint8_t pData[6] = { 0 };
    eSTATUS_t status = SPI_ReadRegister (&(pAccDevice->pBusDevice->spi), BMI3_REG_ACC_DATA_X, pData, 6U);
    RETURN_IF (FJ_FAIL (status), status, "Failed to read bmi323 accelerometer data");

    pOutData[0] = (int16_t)((uint16_t)pData[0] | ((uint16_t)pData[1] << 8U));
    pOutData[1] = (int16_t)((uint16_t)pData[2] | ((uint16_t)pData[3] << 8U));
    pOutData[2] = (int16_t)((uint16_t)pData[4] | ((uint16_t)pData[5] << 8U));

    return eSTATUS_SUCCESS;
}

FJ_STATIC eSTATUS_t Bmi323_ReadGyroData (GYRODevice_t* pGyroDevice, bool forcePolling, int16_t* pOutData) {

    if (!pGyroDevice || !pGyroDevice->pBusDevice || !pOutData) {
        return eSTATUS_NULL_ARG;
    }

    uint16_t timeout = 10000U; // 50 ms timeout
    while (forcePolling && !Bmi323_IsGyroDataReady (&(pGyroDevice->pBusDevice->spi)) && timeout-- > 0) {
        DelayMicroseconds (5);
    }

    uint8_t pData[6] = { 0 };
    eSTATUS_t status = SPI_ReadRegister (&(pGyroDevice->pBusDevice->spi), BMI3_REG_GYR_DATA_X, pData, 6U);
    RETURN_IF (FJ_FAIL (status), status, "Failed to read bmi323 gyroscope data");

    pOutData[0] = (int16_t)((uint16_t)pData[0] | ((uint16_t)pData[1] << 8U));
    pOutData[1] = (int16_t)((uint16_t)pData[2] | ((uint16_t)pData[3] << 8U));
    pOutData[2] = (int16_t)((uint16_t)pData[4] | ((uint16_t)pData[5] << 8U));

    return eSTATUS_SUCCESS;
}

FJ_STATIC eSTATUS_t Bmi323_Init (BusDeviceSPI_t* pBusDeviceSpi) {

    if (g_isBmi323Initialized) {
        return eSTATUS_SUCCESS;
    }

    eSTATUS_t status = Bmi323_SoftReset (pBusDeviceSpi);
    RETURN_IF (FJ_FAIL (status), status, "Failed to soft reset bmi323");
    LOG_INFO ("bmi323 soft reset successful");

    uint8_t remap = 0;
    remap         = BMI3_SET_BIT_POS0 (remap, BMI3_XYZ_AXIS, BMI3_MAP_YXZ_AXIS);
    remap |= BMI3_SET_BITS (remap, BMI3_X_AXIS_SIGN, BMI3_MAP_NEGATIVE);
    remap |= BMI3_SET_BITS (remap, BMI3_Y_AXIS_SIGN, BMI3_MAP_NEGATIVE);
    remap |= BMI3_SET_BITS (remap, BMI3_Z_AXIS_SIGN, BMI3_MAP_NEGATIVE);
    status = Bmi323_RemapAxes (pBusDeviceSpi, remap);
    RETURN_IF (FJ_FAIL (status), status, "Failed to set bmi323 axes remap");
    LOG_INFO ("Successfully remapped bmi323 axes");

    BmiSensorConf_t conf = { 0 };
    conf.odr             = BMI3_ACC_ODR_400HZ;
    conf.range           = BMI3_ACC_RANGE_2G;
    conf.bw              = BMI3_ACC_BW_ODR_HALF;
    conf.avg             = BMI3_ACC_AVG16;
    conf.mode            = BMI3_ACC_MODE_HIGH_PERF;
    status               = SPI_WriteRegister (pBusDeviceSpi, BMI3_REG_ACC_CONF, &conf.raw, 2U);
    RETURN_IF (FJ_FAIL (status), status, "Failed to set bmi323 accelerometer config");
    LOG_INFO ("bmi323 accelerometer configuration successful");

    conf.raw   = 0;
    conf.odr   = BMI3_GYR_ODR_400HZ;
    conf.range = BMI3_GYR_RANGE_250DPS;
    conf.bw    = BMI3_GYR_BW_ODR_HALF;
    conf.avg   = BMI3_GYR_AVG16;
    conf.mode  = BMI3_GYR_MODE_HIGH_PERF;
    status     = SPI_WriteRegister (pBusDeviceSpi, BMI3_REG_GYR_CONF, &conf.raw, 2U);
    RETURN_IF (FJ_FAIL (status), status, "Failed to set bmi323 gyroscope config");
    LOG_INFO ("bmi323 gyroscope configuration successful");

    uint8_t pChipID[2] = { 0 };
    status             = SPI_ReadRegister (pBusDeviceSpi, BMI3_REG_CHIP_ID, pChipID, 2U);
    RETURN_IF (FJ_FAIL (status), status, "Failed to read imu chip id");
    RETURN_IF (pChipID[0] != BMI323_CHIP_ID, eSTATUS_INVALID_ARG, "Unexpected imu chip id");

    status = Bmi323_Calibrate (pBusDeviceSpi);
    RETURN_IF (FJ_FAIL (status), status, "Failed to self calibrate bmi323");
    LOG_INFO ("bmi323 self calibration was successful");

    // TODO: what alternate function do I need for EXTI.
    // Also the NVIC exti interrupt needs to be enabled.
    // EXTI interface also needs to be enabled.
    // GPIO_INIT_EXTI (&status, extiConf.extiId, extiConf.gpioId);
    // /* Enable EXTI interrupt for vIMU_t data ready interrupt */
    g_isBmi323Initialized = true;
    return eSTATUS_SUCCESS;
}

eSTATUS_t Bmi323_InitAcc (AccCfg_t* pAccCfg, ACCDevice_t* pOutAccDevice) {

    if (!pAccCfg || !pOutAccDevice || !pOutAccDevice->pBusDevice) {
        return eSTATUS_NULL_ARG;
    }

    if (pAccCfg->type != INER_INTERFACE_ID_BMI323) {
        LOG_ERROR ("Invalid accelerometer type %d", pAccCfg->type);
        return eSTATUS_INVALID_ARG;
    }

    if (pAccCfg->busCfg.busType != eBUS_TYPE_SPI) {
        LOG_ERROR ("Invalid bus type %d, only SPI is supported", pAccCfg->busCfg.busType);
        return eSTATUS_INVALID_ARG;
    }

    eSTATUS_t status = Bmi323_Init (&pOutAccDevice->pBusDevice->spi);
    RETURN_IF (FJ_FAIL (status), status, "Failed to initialize bmi323 accelerometer");
    // NOTE: scale factor is hardcoded for 2G range and 16-bit resolution
    pOutAccDevice->scaleFactor  = (2.0F * 9.81F) / 32768.0F;
    pOutAccDevice->sampleRateHz = 400;

    pOutAccDevice->vtbl.fnAccReadData = Bmi323_ReadAccData;

    return status;
}

eSTATUS_t Bmi323_InitGyro (GyroCfg_t* pGyroCfg, GYRODevice_t* pOutGyroDevice) {

    if (!pGyroCfg || !pOutGyroDevice || !pOutGyroDevice->pBusDevice) {
        return eSTATUS_NULL_ARG;
    }

    if (pGyroCfg->type != INER_INTERFACE_ID_BMI323) {
        LOG_ERROR ("Invalid gyroscope type %d", pGyroCfg->type);
        return eSTATUS_INVALID_ARG;
    }

    if (pGyroCfg->busCfg.busType != eBUS_TYPE_SPI) {
        LOG_ERROR ("Invalid bus type %d, only SPI is supported", pGyroCfg->busCfg.busType);
        return eSTATUS_INVALID_ARG;
    }

    eSTATUS_t status = Bmi323_Init (&pOutGyroDevice->pBusDevice->spi);
    RETURN_IF (FJ_FAIL (status), status, "Failed to initialize bmi323 gyroscope");
    // NOTE: scale factor is hardcoded for 250 dps range and 16-bit resolution
    pOutGyroDevice->scaleFactor  = 250.0F / 32768.0F;
    pOutGyroDevice->sampleRateHz = 400;

    pOutGyroDevice->vtbl.fnGyroReadData = Bmi323_ReadGyroData;

    return status;
}