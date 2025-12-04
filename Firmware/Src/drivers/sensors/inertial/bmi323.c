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
    uint8_t data    = 0;

    /* Set the configuration to feature engine register */
    eSTATUS_t status = SPI_WriteRegister (pBusDeviceSpi, BMI3_REG_FEATURE_DATA_ADDR, addr, 2U);
    RETURN_IF (FJ_FAIL (status), status, "Failed to set bmi323 feature data address for axis remap");

    data = BMI3_SET_BIT_POS0 (data, BMI3_XYZ_AXIS, remap.remap);
    data |= BMI3_SET_BITS (data, BMI3_X_AXIS_SIGN, remap.xDir);
    data |= BMI3_SET_BITS (data, BMI3_Y_AXIS_SIGN, remap.yDir);
    data |= BMI3_SET_BITS (data, BMI3_Z_AXIS_SIGN, remap.zDir);
    uint8_t aSend[2] = { data, 0 };

    status = IMUWriteReg (pIMU, BMI3_REG_FEATURE_DATA_TX, aSend, 2);
    RETURN_IF (FJ_FAIL (status), status, "Failed to set vIMU_t feature data TX for axis remap");

    /*
     * NOTE: The command to start the axis remap update can be sent without
     * checking the enabled/disabled status of the accel because this
     * function is only called after an vIMU_t soft reset.
     */
    status = IMUSendCmd (pIMU, BMI3_CMD_AXIS_MAP_UPDATE);
    RETURN_IF (FJ_FAIL (status), status, "Failed to send vIMU_t command to update axis remap");

    int16_t wait = 1000;
    status       = eSTATUS_FAILURE;
    while (wait-- > 0) {

        IMU_FeatureReg_t featStatus = { 0 };
        status                      = IMUGetFeatureStatus (pIMU, BMI3_REG_FEATURE_IO1, &featStatus);
        RETURN_IF (FJ_FAIL (status), status, "Failed to get vIMU_t feature status");

        if (AXIS_REMAP_IS_SUCCESSFUL (featStatus)) {
            LOG_INFO ("vIMU_t axis remap successful");
            return eSTATUS_SUCCESS;
        }
        HAL_Delay (1);
    }

    if (FJ_FAIL (status)) {
        LOG_ERROR ("vIMU_t axis remap did not complete in time");
        IMU_LogDeviceErr (pIMU, NULL);
        return status;
    }

    // pIMU->axesRemapConf.remap = remap.remap;
    return eSTATUS_SUCCESS;
}

FJ_STATIC eSTATUS_t Bmi323_Init (BusDeviceSPI_t* pBusDeviceSpi) {

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


    eSTATUS_t status               = eSTATUS_SUCCESS;
    IMUAccConf accConf             = conf.aconf;
    IMUGyroConf gyroConf           = conf.gconf;
    IMUAxesRemapConf axesRemapConf = conf.axesRemapConf;
    DevDesc_t* pDevDesc            = conf.pDevDesc;

    odr     = BMI3_SET_BIT_POS0 (pRegData[0], BMI3_ACC_ODR, pAConf->odr);
    range   = BMI3_SET_BITS (pRegData[0], BMI3_ACC_RANGE, pAConf->range);
    bwp     = BMI3_SET_BITS (pRegData[0], BMI3_ACC_BW, pAConf->bw);
    avgNum  = BMI3_SET_BITS (pRegData[1], BMI3_ACC_AVG_NUM, pAConf->avg);
    accMode = BMI3_SET_BITS (pRegData[1], BMI3_ACC_MODE, pAConf->mode);

    vIMU_t* pIMU = &gIMU;
    if (pOutIMU != NULL) {
        pIMU = pOutIMU;
    }
    memset (pIMU, 0, sizeof (vIMU_t));
    pIMU->deviceId = DEV_DESC_GET_ID (pDevDesc);
    pIMU->aconf    = accConf;
    pIMU->gconf    = gyroConf;
    pIMU->bus      = *conf.pBus;
    /* SPI reads have 1 dummy byte at the beginning */
    pIMU->nBusDummyBytes = BUS_IS_SPI (&pIMU->bus) ? 1 : 0;

    /*
     * Soft reset vIMU_t and switch to SPI
     */
    status = IMUSoftReset (pIMU);
    GOTO_IF (FJ_FAIL (status), error, "Failed to soft reset imu");
    LOG_INFO ("IMU soft reset successful");

    status = IMUSetAxesRemap (pIMU, axesRemapConf);
    GOTO_IF (FJ_FAIL (status), error, "Failed to set imu axes remap");
    LOG_INFO ("Successfully remapped imu axes");

    /*
     * Setup the accel and gyro using the provided configurations
     */
    status = IMUSetConf (pIMU, &accConf, &gyroConf);
    GOTO_IF (FJ_FAIL (status), error, "Failed to set imu config");
    LOG_INFO ("IMU configuration successful");
    IMU_LogDeviceConf (pIMU);

    uint8_t pChipID[2] = { 0 };
    status             = IMUReadReg (pIMU, BMI3_REG_CHIP_ID, pChipID, 2U);
    GOTO_IF (FJ_FAIL (status), error, "Failed to read imu chip id");
    GOTO_IF (pChipID[0] != IMU_CHIP_ID, error, "Unexpected imu chip id");

    /* Self Calibrate */
    status = IMUCalibrate (pIMU, BMI3_SC_SENSITIVITY_EN | BMI3_SC_OFFSET_EN, BMI3_SC_APPLY_CORR_EN);
    GOTO_IF (FJ_FAIL (status), error, "Failed to self calibrate IMU");
    LOG_INFO ("IMU self calibration was successful");
    IMU_LogDeviceConf (pIMU);

    {
        IMUAccConf aconf2;
        IMUGyroConf gconf2;
        status = IMUGetConf (pIMU, &aconf2, &gconf2);
        if (status != eSTATUS_SUCCESS) {
            LOG_ERROR ("Failed to read back IMU configuration");
            IMU_LogError (pIMU);
            goto error;
        }

        status = IMUCompareConfs (accConf, gyroConf, aconf2, gconf2);
        if (status != eSTATUS_SUCCESS) {
            LOG_ERROR (
            "vIMU_t configuration mismatch after setting. "
            "Expected: Accel [%d %d %d %d] Gyro [%d %d %d %d] "
            "Got: Accel [%d %d %d %d] Gyro [%d %d %d %d]",
            accConf.mode,
            accConf.odr,
            accConf.range,
            accConf.avg,
            gyroConf.mode,
            gyroConf.odr,
            gyroConf.range,
            gyroConf.avg,
            aconf2.mode,
            aconf2.odr,
            aconf2.range,
            aconf2.avg,
            gconf2.mode,
            gconf2.odr,
            gconf2.range,
            gconf2.avg
            );
            IMU_LogError (pIMU);
            goto error;
        }
    }

    // TODO: what alternate function do I need for EXTI.
    // Also the NVIC exti interrupt needs to be enabled.
    // EXTI interface also needs to be enabled.
    // GPIO_INIT_EXTI (&status, extiConf.extiId, extiConf.gpioId);
    // /* Enable EXTI interrupt for vIMU_t data ready interrupt */
    return eSTATUS_SUCCESS;
}

eSTATUS_t Bmi323_InitAcc (AccCfg_t* pAccCfg, ACCDevice_t* pOutAccDevice) {

    if (!pAccCfg || !pOutAccDevice) {
        return eSTATUS_NULL_ARG;
    }

    if (pAccCfg->type != INER_TYPE_BMI323) {
        LOG_ERROR ("Bmi323_InitAcc: Invalid accelerometer type %d", pAccCfg->type);
        return eSTATUS_INVALID_ARG;
    }

    if (pAccCfg->busCfg.busType != eBUS_TYPE_SPI) {
        LOG_ERROR ("Bmi323_InitAcc: Invalid bus type %d, only SPI is supported", pAccCfg->busCfg.busType);
        return eSTATUS_INVALID_ARG;
    }
    return eSTATUS_SUCCESS;
}

eSTATUS_t Bmi323_InitGyro (GyroCfg_t* pGyroCfg, GYRODevice_t* pOutGyroDevice) {
}