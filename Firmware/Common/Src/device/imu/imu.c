#include "device/imu/imu.h"
#include "core/core.h"
#include "core/log/logger.h"
#include "device/imu/bmixxx.h"
#include "mem/mem.h"
#include "target.h"


#include "drivers/bus/spi.h"

#include "drivers/io/gpio.h"

#include "target.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>


#define IMU_CHIP_ID            ((uint8_t)0x0043U)
#define RW_BUFFER_SZ           16U
#define SPI_DEFAULT_TIMEOUT_MS 100U
#define IMU_VALID(pIMU)        ((pIMU) != NULL && (pIMU)->isInitialized == true)

#define CALIB_IS_ONGOING(FEAT_REG) \
    ((FEAT_REG).feat_1.sc_st_complete == eBMI3_FEAT_CALIB_ONGOING && (FEAT_REG).feat_1.state == eBMI3_FEAT_STATE_CALIB_ONGOING)

#define CALIB_IS_SUCCESSFUL(FEAT_REG)                                                                                           \
    (                                                                                                                           \
    (FEAT_REG).feat_1.sc_st_complete == eBMI3_FEAT_CALIB_COMPLETE &&                                                            \
    (FEAT_REG).feat_1.error_status == eBMI3_FEAT_ERROR_NO_ERROR && (FEAT_REG).feat_1.state == eBMI3_FEAT_STATE_SYS_IN_FEAT_MODE \
    )

#define AXIS_REMAP_IS_SUCCESSFUL(FEAT_REG)                                                                                      \
    (                                                                                                                           \
    (FEAT_REG).feat_1.axis_map_complete == eBMI3_FEAT_AXIS_MAP_COMPLETE &&                                                      \
    (FEAT_REG).feat_1.error_status == eBMI3_FEAT_ERROR_NO_ERROR && (FEAT_REG).feat_1.state == eBMI3_FEAT_STATE_SYS_IN_FEAT_MODE \
    )

#define INT_STATUS_HAS_ERROR(INT_STATUS_REG)        ((INT_STATUS_REG).int_1.errStatus != 0U)
#define INT_STATUS_ACCEL_DATA_READY(INT_STATUS_REG) ((INT_STATUS_REG).int_1.drdyAccel != 0U)
#define INT_STATUS_GYRO_DATA_READY(INT_STATUS_REG)  ((INT_STATUS_REG).int_1.drdyGyro != 0U)
#define INT_STATUS_TEMP_DATA_READY(INT_STATUS_REG)  ((INT_STATUS_REG).int_1.drdyTemp != 0U)

#define SYS_STATUS_ACCEL_DATA_READY(SYS_STATUS_REG) ((SYS_STATUS_REG).drdyAccel != 0U)
#define SYS_STATUS_GYRO_DATA_READY(SYS_STATUS_REG)  ((SYS_STATUS_REG).drdyGyro != 0U)
#define SYS_STATUS_TEMP_DATA_READY(SYS_STATUS_REG)  ((SYS_STATUS_REG).drdyTemp != 0U)

FJ_DEFINE_SHARED (IMU_t, g_IMU);

static eSTATUS_t IMUGetDeviceErr (vIMU_t* pIMU, IMUErr* pOutErr);
static void IMU_LogFeatStatus (vIMU_t* pIMU);
static void IMU_LogDeviceErr (vIMU_t* pIMU, IMUErr const* pErr);
static void IMU_LogDeviceConf (vIMU_t* pIMU);
static void IMU_LogError (vIMU_t* pIMU);

#ifndef UNIT_TEST

static eSTATUS_t IMUSendCmd (vIMU_t* pIMU, uint16_t cmd);
static eSTATUS_t IMUGetFeatureStatus (vIMU_t* pIMU, uint16_t featureRegAddr, IMU_FeatureReg_t* pOutStatus);
static eSTATUS_t IMUGetINTStatus (vIMU_t* pIMU, IMU_INTStatusReg_t* pOutStatus);
static eSTATUS_t IMUGetSysStatus (vIMU_t* pIMU, IMU_SysStatusReg_t* pOutStatus);
static eSTATUS_t IMUReadReg (vIMU_t* pIMU, uint8_t reg, uint8_t* pBuf, uint32_t len);
static eSTATUS_t IMUWriteReg (vIMU_t* pIMU, uint8_t reg, uint8_t* pBuf, uint32_t len);
static eSTATUS_t IMUUpdateRawGyro (vIMU_t* pIMU);
static eSTATUS_t IMUUpdateRawAccel (vIMU_t* pIMU);
static eSTATUS_t IMUSetAxesRemap (vIMU_t* pIMU, IMUAxesRemapConf remap);
static eSTATUS_t IMUSoftReset (vIMU_t* pIMU);
static eSTATUS_t IMUGetConf_ (vIMU_t* pIMU, IMUAccConf* pAConf, IMUGyroConf* pGConf, uint8_t altConfFlag);
static eSTATUS_t IMUSetConf_ (vIMU_t* pIMU, IMUAccConf const* pAConf, IMUGyroConf const* pGConf, uint8_t altConfFlag);
static eSTATUS_t IMUCalibrate (vIMU_t* pIMU, uint8_t calibSelection, uint8_t applyCorrection);
static eSTATUS_t IMUSetupInterrupts (vIMU_t* pIMU);
// static eSTATUS_t IMUEnableInterrupts (vIMU_t* pIMU);
static eSTATUS_t IMUDisableInterrupts (vIMU_t* pIMU);
static eSTATUS_t
IMUConvertRaw (IMU_ACC_RANGE aRange, Vec3i ra, IMU_GYRO_RANGE gRange, Vec3i rg, Vec3f* pAccelOut, Vec3f* pGyroOut);

#endif

static void IMU_LogFeatStatus (vIMU_t* pIMU) {

    IMU_FeatureReg_t featStatus = { 0 };
    eSTATUS_t status            = IMUGetFeatureStatus (pIMU, BMI3_REG_FEATURE_IO1, &featStatus);
    if (STATUS_FAIL (status)) {
        LOG_ERROR ("Failed to read vIMU_t feature status register");
        return;
    }

    LOG_INFO (
    "vIMU_t Feature Status: error_status=%u, sc_st_complete=%u, gyro_sc_complete=%u, st_result=%u, "
    "sample_rate_err=%u, axis_map_complete=%u, state=%u",
    featStatus.feat_1.error_status,
    featStatus.feat_1.sc_st_complete,
    featStatus.feat_1.gyro_sc_complete,
    featStatus.feat_1.st_result,
    featStatus.feat_1.sample_rate_err,
    featStatus.feat_1.axis_map_complete,
    featStatus.feat_1.state
    );
}

static void IMU_LogDeviceConf (vIMU_t* pIMU) {

    IMUAccConf aConf  = { 0 };
    IMUGyroConf gConf = { 0 };
    eSTATUS_t status  = IMUGetConf (pIMU, &aConf, &gConf);

    if (STATUS_FAIL (status)) {
        LOG_ERROR ("Failed to get IMU configuration");
        return;
    }

    LOG_INFO (
    "vIMU_t Configuration: Accel Range=%u, Accel ODR=%u, Accel BW=%u, Accel AVG=%u, Accel Mode=%u; "
    "Gyro Range=%u, Gyro ODR=%u, Gyro BW=%u, Gyro AVG=%u, Gyro Mode=%u",
    aConf.range,
    aConf.odr,
    aConf.bw,
    aConf.avg,
    aConf.mode,
    gConf.range,
    gConf.odr,
    gConf.bw,
    gConf.avg,
    gConf.mode
    );
}

void IMU_LogDeviceErr (vIMU_t* pIMU, IMUErr const* pErr) {

    IMUErr err;
    if (pErr == NULL) {
        if (IMUGetDeviceErr (pIMU, &err) != eSTATUS_SUCCESS) {
            LOG_ERROR ("Failed to read vIMU_t error codes");
            return;
        }
    } else {
        err = *pErr;
    }

    if (err.fatalErr != 0) {
        LOG_ERROR (
        "vIMU_t fatal error, chip is not in operation state (Boot or "
        "Power-System). Power on reset or soft reset required"
        );
    }
    if (err.featEngOvrld != 0) {
        LOG_ERROR ("vIMU_t overload of the feature engine detected");
    }
    if (err.featEngWd != 0) {
        LOG_ERROR ("vIMU_t watchdog timer of the feature engine triggered");
    }
    if (err.accConfErr != 0) {
        LOG_ERROR ("vIMU_t unsupported accelerometer configuration set by user");
    }
    if (err.gyrConfErr != 0) {
        LOG_ERROR ("vIMU_t unsupported gyroscope configuration set by user");
    }
    if (err.i3cErr0 != 0) {
        LOG_ERROR ("vIMU_t I3C SDR parity error occurred");
    }
    if (err.i3cErr1 != 0) {
        LOG_ERROR ("vIMU_t I3C S0/S1 error occurred");
    } else {
        LOG_ERROR ("Did NOT find any vIMU_t device errors");
    }
}

static void IMU_LogError (vIMU_t* pIMU) {
    IMU_LogFeatStatus (pIMU);
    IMU_LogDeviceConf (pIMU);
    IMU_LogDeviceErr (pIMU, NULL);
}

STATIC eSTATUS_t IMUSendCmd (vIMU_t* pIMU, uint16_t cmd) {

    uint8_t pRegData[2] = { 0 };
    pRegData[0]         = (uint8_t)(cmd & BMI3_SET_LOW_BYTE);
    pRegData[1]         = (uint8_t)((cmd & BMI3_SET_HIGH_BYTE) >> 8U);
    eSTATUS_t status    = IMUWriteReg (pIMU, BMI3_REG_CMD, pRegData, 2);
    if (status != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to send vIMU_t command 0x%04X", cmd);
        return status;
    }
    return eSTATUS_SUCCESS;
}

STATIC eSTATUS_t IMUGetFeatureStatus (vIMU_t* pIMU, uint16_t featureRegAddr, IMU_FeatureReg_t* pOutStatus) {

    uint8_t pData[2] = { 0 };
    eSTATUS_t status = IMUReadReg (pIMU, featureRegAddr, pData, 2U);
    if (STATUS_FAIL (status)) {
        LOG_ERROR ("Failed to read IMU feature status register [0x%04X]", featureRegAddr);
        return status;
    }

    pOutStatus->raw = BUF_TO_U16 (pData);
    if (featureRegAddr == BMI3_REG_FEATURE_IO1) {
        return eSTATUS_SUCCESS;
    }

    LOG_ERROR ("Trying read from unsupported feature status register [0x%04X]", featureRegAddr);
    return eSTATUS_FAILURE;
}

STATIC eSTATUS_t IMUGetINTStatus (vIMU_t* pIMU, IMU_INTStatusReg_t* pOutStatus) {

    uint8_t pBuff[2] = { 0U };
    eSTATUS_t status = IMUReadReg (pIMU, BMI3_REG_INT_STATUS_INT1, pBuff, 2);
    if (status != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to read vIMU_t interrupt status register");
        return status;
    }
    pOutStatus->raw = BUF_TO_U16 (pBuff);
    return eSTATUS_SUCCESS;
}

STATIC eSTATUS_t IMUGetSysStatus (vIMU_t* pIMU, IMU_SysStatusReg_t* pOutStatus) {

    uint8_t pBuff[2] = { 0U };
    eSTATUS_t status = IMUReadReg (pIMU, BMI3_REG_STATUS, pBuff, 2);
    if (status != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to read vIMU_t status register");
        return status;
    }
    pOutStatus->raw = BUF_TO_U16 (pBuff);
    return eSTATUS_SUCCESS;
}

STATIC eSTATUS_t IMUGetDeviceErr (vIMU_t* pIMU, IMUErr* pOutErr) {

    uint8_t pBuff[2]      = { 0U };
    eSTATUS_t status      = IMUReadReg (pIMU, BMI3_REG_ERR_REG, pBuff, 2);
    uint16_t err          = ((uint16_t)pBuff[1] << 8U) | (uint16_t)pBuff[0];
    pOutErr->err          = err;
    pOutErr->fatalErr     = (err & (1U << 0U)) > 0;
    pOutErr->featEngOvrld = (err & (1U << 2U)) > 0;
    pOutErr->featEngWd    = (err & (1U << 4U)) > 0;
    pOutErr->accConfErr   = (err & (1U << 5U)) > 0;
    pOutErr->gyrConfErr   = (err & (1U << 6U)) > 0;
    pOutErr->i3cErr0      = (err & (1U << 8U)) > 0;
    pOutErr->i3cErr1      = (err & (1U << 11U)) > 0;
    if (status != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to read vIMU_t error register");
        return status;
    }
    return eSTATUS_SUCCESS;
}

STATIC eSTATUS_t IMUReadReg (vIMU_t* pIMU, uint8_t reg, uint8_t* pBuf, uint32_t len) {

    eSTATUS_t status = SpiDev_ReadRegister (&pIMU->spiDev, reg, pBuf, len);
    if (status != eSTATUS_SUCCESS) {
        return (eSTATUS_t)eIMU_COM_FAILURE;
    }
    // Add 2 microsecond delay after vIMU_t read operation
    DelayMicroseconds (2);
    return eSTATUS_SUCCESS;
}

STATIC eSTATUS_t IMUWriteReg (vIMU_t* pIMU, uint8_t reg, uint8_t* pBuf, uint32_t len) {

    eSTATUS_t status = SpiDev_WriteRegister (&pIMU->spiDev, reg, pBuf, len);
    if (status != eSTATUS_SUCCESS) {
        return (eSTATUS_t)eIMU_COM_FAILURE;
    }
    // Add 2 microsecond delay after vIMU_t write operation
    DelayMicroseconds (2);
    return eSTATUS_SUCCESS;
}

STATIC eSTATUS_t IMUUpdateRawGyro (vIMU_t* pIMU) {

    uint8_t pBuffer[6] = { 0 };
    eSTATUS_t status   = IMUReadReg (pIMU, BMI3_REG_GYR_DATA_X, pBuffer, 6);
    if (status != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to read vIMU_t gyroscope data registers");
        IMU_LogError (pIMU);
        return status;
    }

    pIMU->rawGyro.x = (int16_t)((((uint16_t)pBuffer[1]) << 8U) | ((uint16_t)pBuffer[0]));
    pIMU->rawGyro.y = (int16_t)((((uint16_t)pBuffer[3]) << 8U) | ((uint16_t)pBuffer[2]));
    pIMU->rawGyro.z = (int16_t)((((uint16_t)pBuffer[5]) << 8U) | ((uint16_t)pBuffer[4]));

    pIMU->gyroDataUpdated      = true;
    pIMU->msLastGyroUpdateTime = GetMilliseconds ();
    return eSTATUS_SUCCESS;
}

STATIC eSTATUS_t IMUUpdateRawAccel (vIMU_t* pIMU) {

    uint8_t pBuffer[6] = { 0 };
    eSTATUS_t status   = IMUReadReg (pIMU, BMI3_REG_ACC_DATA_X, pBuffer, 6);
    if (status != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to read vIMU_t gyroscope data registers");
        IMU_LogError (pIMU);
        return status;
    }

    pIMU->rawAccel.x = (int16_t)((((uint16_t)pBuffer[1]) << 8U) | ((uint16_t)pBuffer[0]));
    pIMU->rawAccel.y = (int16_t)((((uint16_t)pBuffer[3]) << 8U) | ((uint16_t)pBuffer[2]));
    pIMU->rawAccel.z = (int16_t)((((uint16_t)pBuffer[5]) << 8U) | ((uint16_t)pBuffer[4]));

    pIMU->accelDataUpdated    = true;
    pIMU->msLastAccUpdateTime = GetMilliseconds ();
    return eSTATUS_SUCCESS;
}

STATIC eSTATUS_t IMUSetAxesRemap (vIMU_t* pIMU, IMUAxesRemapConf remap) {

    uint8_t addr[2] = { BMI3_BASE_ADDR_AXIS_REMAP, 0 };
    uint8_t data    = 0;

    /* Set the configuration to feature engine register */
    eSTATUS_t status = IMUWriteReg (pIMU, BMI3_REG_FEATURE_DATA_ADDR, addr, 2U);
    RETURN_IF (STATUS_FAIL (status), status, "Failed to set vIMU_t feature data address for axis remap");

    data = BMI3_SET_BIT_POS0 (data, BMI3_XYZ_AXIS, remap.remap);
    data |= BMI3_SET_BITS (data, BMI3_X_AXIS_SIGN, remap.xDir);
    data |= BMI3_SET_BITS (data, BMI3_Y_AXIS_SIGN, remap.yDir);
    data |= BMI3_SET_BITS (data, BMI3_Z_AXIS_SIGN, remap.zDir);
    uint8_t aSend[2] = { data, 0 };

    status = IMUWriteReg (pIMU, BMI3_REG_FEATURE_DATA_TX, aSend, 2);
    RETURN_IF (STATUS_FAIL (status), status, "Failed to set vIMU_t feature data TX for axis remap");

    /*
     * NOTE: The command to start the axis remap update can be sent without
     * checking the enabled/disabled status of the accel because this
     * function is only called after an vIMU_t soft reset.
     */
    status = IMUSendCmd (pIMU, BMI3_CMD_AXIS_MAP_UPDATE);
    RETURN_IF (STATUS_FAIL (status), status, "Failed to send vIMU_t command to update axis remap");

    int16_t wait = 1000;
    status       = eSTATUS_FAILURE;
    while (wait-- > 0) {

        IMU_FeatureReg_t featStatus = { 0 };
        status                      = IMUGetFeatureStatus (pIMU, BMI3_REG_FEATURE_IO1, &featStatus);
        RETURN_IF (STATUS_FAIL (status), status, "Failed to get vIMU_t feature status");

        if (AXIS_REMAP_IS_SUCCESSFUL (featStatus)) {
            LOG_INFO ("vIMU_t axis remap successful");
            return eSTATUS_SUCCESS;
        }
        HAL_Delay (1);
    }

    if (STATUS_FAIL (status)) {
        LOG_ERROR ("vIMU_t axis remap did not complete in time");
        IMU_LogDeviceErr (pIMU, NULL);
        return status;
    }

    // pIMU->axesRemapConf.remap = remap.remap;
    return eSTATUS_SUCCESS;
}

STATIC eSTATUS_t IMUSoftReset (vIMU_t* pIMU) {
    /* Send soft reset command to BMI323 */
    eSTATUS_t status = IMUSendCmd (pIMU, BMI3_CMD_SOFT_RESET);

    if (status != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to send soft reset command to vIMU_t");
        return status;
    }

    HAL_Delay (100);
    /* Perform dummy read to switch from I3C/I2C to SPI */
    if (status == eSTATUS_SUCCESS) {
        uint8_t dummyBytes[2] = { 0 };
        status                = IMUReadReg (pIMU, BMI3_REG_CHIP_ID, dummyBytes, 2);
    }

    HAL_Delay (100);
    /* Enable feature engine */
    if (status == eSTATUS_SUCCESS) {
        uint8_t featureData[2] = { 0x2C, 0x01 };
        status                 = IMUWriteReg (pIMU, BMI3_REG_FEATURE_IO2, featureData, 2);
    }

    HAL_Delay (100);
    /* Enable feature status bit */
    if (status == eSTATUS_SUCCESS) {
        uint8_t featureIOStatus[2] = { BMI3_ENABLE, 0 };
        status = IMUWriteReg (pIMU, BMI3_REG_FEATURE_IO_STATUS, featureIOStatus, 2);
    }

    HAL_Delay (100);
    /* Enable feature engine bit */
    if (status == eSTATUS_SUCCESS) {
        uint8_t featureEngine[2] = { BMI3_ENABLE, 0 };
        status                   = IMUWriteReg (pIMU, BMI3_REG_FEATURE_CTRL, featureEngine, 2);
    }

    uint8_t featEnabled = false;
    if (status == eSTATUS_SUCCESS) {
        int16_t loop       = 0;
        uint8_t regData[2] = { 0 };

        while (loop++ <= 20) {
            HAL_Delay (100);
            status = IMUReadReg (pIMU, BMI3_REG_FEATURE_IO1, regData, 2);
            if (status == eSTATUS_SUCCESS) {
                if (regData[0] & (uint16_t)BMI3_FEATURE_ENGINE_ENABLE_MASK) {
                    featEnabled = true;
                    break;
                }
            }
        }
    }
    if (featEnabled != true) {
        LOG_ERROR ("Failed to enable feature engine after soft reset");
        IMU_LogDeviceErr (pIMU, NULL);
        return eSTATUS_FAILURE;
    }
    return status;
}

STATIC eSTATUS_t IMUGetConf_ (vIMU_t* pIMU, IMUAccConf* pAConf, IMUGyroConf* pGConf, uint8_t altConfFlag) {
    eSTATUS_t status = eSTATUS_SUCCESS;
    /* Accelerometer Config */
    if (pAConf != NULL) {
        uint8_t regAddr = BMI3_REG_ACC_CONF;
        if (altConfFlag == 1) {
            regAddr = BMI3_REG_ALT_ACC_CONF;
        }

        uint8_t data[2] = { 0 };
        status          = IMUReadReg (pIMU, regAddr, data, 2);
        if (status != eSTATUS_SUCCESS) {
            return status;
        }

        uint16_t conf = data[0];
        // NOLINTBEGIN(hicpp-signed-bitwise)
        pAConf->odr   = BMI3_GET_BIT_POS0 (conf, BMI3_ACC_ODR);
        pAConf->range = BMI3_GET_BITS (conf, BMI3_ACC_RANGE);
        pAConf->bw    = BMI3_GET_BITS (conf, BMI3_ACC_BW);

        conf         = ((uint16_t)data[1]) << 8U;
        pAConf->avg  = BMI3_GET_BITS (conf, BMI3_ACC_AVG_NUM);
        pAConf->mode = BMI3_GET_BITS (conf, BMI3_ACC_MODE);

        if (altConfFlag == 1) {
            conf        = data[0];
            pAConf->odr = BMI3_GET_BIT_POS0 (conf, BMI3_ALT_ACC_ODR);

            conf         = ((uint16_t)data[1]) << 8U;
            pAConf->avg  = BMI3_GET_BITS (conf, BMI3_ALT_ACC_AVG_NUM);
            pAConf->mode = BMI3_GET_BITS (conf, BMI3_ALT_ACC_MODE);
        }
        // NOLINTEND(hicpp-signed-bitwise)
    }

    /* Gyro Config */
    if (pGConf != NULL) {
        uint8_t regAddr = BMI3_REG_GYR_CONF;
        if (altConfFlag == 1) {
            regAddr = BMI3_REG_ALT_GYR_CONF;
        }
        uint8_t data[2] = { 0 };
        status          = IMUReadReg (pIMU, regAddr, data, 2);
        if (status != eSTATUS_SUCCESS) {
            return status;
        }
        uint16_t conf = data[0];
        // NOLINTBEGIN(hicpp-signed-bitwise)
        pGConf->odr   = BMI3_GET_BIT_POS0 (conf, BMI3_GYR_ODR);
        pGConf->range = BMI3_GET_BITS (conf, BMI3_GYR_RANGE);
        pGConf->bw    = BMI3_GET_BITS (conf, BMI3_GYR_BW);

        conf         = ((uint16_t)data[1]) << 8U;
        pGConf->avg  = BMI3_GET_BITS (conf, BMI3_GYR_AVG_NUM);
        pGConf->mode = BMI3_GET_BITS (conf, BMI3_GYR_MODE);

        if (altConfFlag == 1) {
            conf        = data[0];
            pGConf->odr = BMI3_GET_BIT_POS0 (conf, BMI3_ALT_GYR_ODR);

            conf         = ((uint16_t)data[1]) << 8U;
            pGConf->avg  = BMI3_GET_BITS (conf, BMI3_ALT_GYR_AVG_NUM);
            pGConf->mode = BMI3_GET_BITS (conf, BMI3_ALT_GYR_MODE);
        }
        // NOLINTEND(hicpp-signed-bitwise)
    }
    return status;
}

STATIC eSTATUS_t IMUSetConf_ (vIMU_t* pIMU, IMUAccConf const* pAConf, IMUGyroConf const* pGConf, uint8_t altConfFlag) {
    /* Configure Accelerometer */
    eSTATUS_t status = eSTATUS_SUCCESS;
    if (pAConf != NULL) {
        uint8_t pRegData[2] = { 0 };
        uint16_t odr        = 0;
        uint16_t range      = 0;
        uint16_t bwp        = 0;
        uint16_t avgNum     = 0;
        uint16_t accMode    = 0;
        uint8_t regAddr     = BMI3_REG_ACC_CONF;
        // NOLINTBEGIN(*)
        odr     = BMI3_SET_BIT_POS0 (pRegData[0], BMI3_ACC_ODR, pAConf->odr);
        range   = BMI3_SET_BITS (pRegData[0], BMI3_ACC_RANGE, pAConf->range);
        bwp     = BMI3_SET_BITS (pRegData[0], BMI3_ACC_BW, pAConf->bw);
        avgNum  = BMI3_SET_BITS (pRegData[1], BMI3_ACC_AVG_NUM, pAConf->avg);
        accMode = BMI3_SET_BITS (pRegData[1], BMI3_ACC_MODE, pAConf->mode);

        if (altConfFlag == true) {
            regAddr = BMI3_REG_ALT_ACC_CONF;
            odr     = BMI3_SET_BIT_POS0 (pRegData[0], BMI3_ALT_ACC_ODR, pAConf->odr);
            avgNum  = BMI3_SET_BITS (pRegData[1], BMI3_ALT_ACC_AVG_NUM, pAConf->avg);
            accMode = BMI3_SET_BITS (pRegData[1], BMI3_ALT_ACC_MODE, pAConf->mode);
        }

        pRegData[0] = (uint8_t)(odr | range | bwp);
        pRegData[1] = (uint8_t)((avgNum | accMode) >> 8U);
        status      = IMUWriteReg (pIMU, regAddr, pRegData, 2);
        // NOLINTEND(*)
        if (status != eSTATUS_SUCCESS) {
            // LOG_ERROR ("Failed to configure vIMU_t accelerometer");
            return status;
        }
        if (status == eSTATUS_SUCCESS && altConfFlag == false) {
            pIMU->aconf = *pAConf;
        }
    }

    /* Configure Gyro */
    if (pGConf != NULL) {
        uint8_t pRegData[2] = { 0 };
        uint16_t odr        = 0;
        uint16_t range      = 0;
        uint16_t bwp        = 0;
        uint16_t avgNum     = 0;
        uint16_t accMode    = 0;
        uint8_t regAddr     = BMI3_REG_GYR_CONF;
        // NOLINTBEGIN(*)
        odr     = BMI3_SET_BIT_POS0 (pRegData[0], BMI3_GYR_ODR, pGConf->odr);
        range   = BMI3_SET_BITS (pRegData[0], BMI3_GYR_RANGE, pGConf->range);
        bwp     = BMI3_SET_BITS (pRegData[0], BMI3_GYR_BW, pGConf->bw);
        avgNum  = BMI3_SET_BITS (pRegData[1], BMI3_GYR_AVG_NUM, pGConf->avg);
        accMode = BMI3_SET_BITS (pRegData[1], BMI3_GYR_MODE, pGConf->mode);

        if (altConfFlag == true) {
            regAddr = BMI3_REG_ALT_GYR_CONF;
            odr     = BMI3_SET_BIT_POS0 (pRegData[0], BMI3_ALT_GYR_ODR, pGConf->odr);
            avgNum  = BMI3_SET_BITS (pRegData[1], BMI3_ALT_GYR_AVG_NUM, pGConf->avg);
            accMode = BMI3_SET_BITS (pRegData[1], BMI3_ALT_GYR_MODE, pGConf->mode);
        }

        pRegData[0] = (uint8_t)(odr | range | bwp);
        pRegData[1] = (uint8_t)((avgNum | accMode) >> 8U);
        status      = IMUWriteReg (pIMU, regAddr, pRegData, 2);
        // NOLINTEND(*)
        if (status != eSTATUS_SUCCESS) {
            LOG_ERROR ("Failed to configure vIMU_t gyroscope");
            return eSTATUS_FAILURE;
        }
        if (status == eSTATUS_SUCCESS && altConfFlag == false) {
            pIMU->gconf = *pGConf;
        }
    }
    return status;
}

STATIC eSTATUS_t IMUCalibrate (vIMU_t* pIMU, uint8_t calibSelection, uint8_t applyCorrection) {

    FJ_UNUSED (calibSelection);
    FJ_UNUSED (applyCorrection);

    /* Save the current configs */
    IMUAccConf aconf;
    IMUGyroConf gconf;
    eSTATUS_t status = IMUGetConf (pIMU, &aconf, &gconf);
    RETURN_IF (STATUS_FAIL (status), status, "Failed to get IMU configuration to save before calibration");
    HAL_Delay (100);

    /* Set the ACC config to be what the self calibration expects */
    IMUAccConf calibAConf = { 0 };
    calibAConf.mode       = eIMU_ACC_MODE_HIGH_PERF;
    calibAConf.odr        = eIMU_ACC_ODR_100;
    calibAConf.range      = aconf.range;
    calibAConf.avg        = aconf.avg;
    calibAConf.bw         = aconf.bw;
    status                = IMUSetConf (pIMU, &calibAConf, NULL);
    RETURN_IF (STATUS_FAIL (status), status, "Failed to set IMU accelerometer configuration for calibration");
    HAL_Delay (100);

    /* Store alt configs and then disable them */
    IMUAccConf altAConf;
    IMUGyroConf altGConf;
    status = IMUGetAltConf (pIMU, &altAConf, &altGConf);
    GOTO_IF (STATUS_FAIL (status), error, "Failed to get IMU alternate configuration to save before calibration");

    altAConf.mode = eIMU_ACC_MODE_DISABLE;
    altGConf.mode = eIMU_GYRO_MODE_DISABLE;
    status        = IMUSetAltConf (pIMU, &altAConf, &altGConf);
    GOTO_IF (STATUS_FAIL (status), error, "Failed to disable IMU alternate configuration before calibration");
    HAL_Delay (100);

    /* Trigger the self calibration */
    status = IMUSendCmd (pIMU, BMI3_CMD_SELF_CALIB_TRIGGER);
    GOTO_IF (STATUS_FAIL (status), error, "Failed to send IMU self-calibration trigger command");
    HAL_Delay (100);

    /* Check that the self calibration has started */
    {
        IMU_FeatureReg_t featureStatus = { 0 };
        status = IMUGetFeatureStatus (pIMU, BMI3_REG_FEATURE_IO1, &featureStatus);
        GOTO_IF (STATUS_FAIL (status), error, "Failed to get IMU feature status");

        if (CALIB_IS_ONGOING (featureStatus) == false) {
            LOG_ERROR ("IMU has not started the self calibration");
            IMU_LogDeviceErr (pIMU, NULL);
            goto error;
        }
    }

    /* Get self calibration result */
    {
        for (uint8_t idx = 0; idx < 10U; idx++) {
            /* A delay of 1000ms (100ms * 10(limit)) is required to perform self calibration */
            HAL_Delay (100);
            IMU_INTStatusReg_t INTStatus = { 0 };
            status                       = IMUGetINTStatus (pIMU, &INTStatus);
            GOTO_IF (STATUS_FAIL (status), error, "Failed to get IMU interrupt status during self-calibration");

            if (INT_STATUS_HAS_ERROR (INTStatus)) {
                break;
            }
        }

        IMU_FeatureReg_t featureStatus = { 0 };
        status = IMUGetFeatureStatus (pIMU, BMI3_REG_FEATURE_IO1, &featureStatus);
        GOTO_IF (STATUS_FAIL (status), error, "Failed to get IMU feature status after self-calibration");

        if (CALIB_IS_SUCCESSFUL (featureStatus) == false) {
            LOG_ERROR ("IMU self-calibration failed");
            IMU_LogError (pIMU);
        }
    }
    HAL_Delay (20);
/* Restore configs */
error:
    status = IMUSetConf (pIMU, &aconf, &gconf);
    status = IMUSetAltConf (pIMU, &altAConf, &altGConf);
    HAL_Delay (100);
    return status;
}

STATIC eSTATUS_t IMUSetupInterrupts (vIMU_t* pIMU) {
    uint8_t pRegData[4] = { 0 };
    uint16_t temp       = 0;
    /* Map all enabled interrupts to pin INT1 */
    uint8_t enable   = BMI3_INT1;
    uint8_t disable  = BMI3_INT_NONE;
    eSTATUS_t status = IMUReadReg (pIMU, BMI3_REG_INT_MAP1, pRegData, 4);

    if (status == eSTATUS_SUCCESS) {
        // NOLINTBEGIN(*)
        pRegData[0] = BMI3_SET_BIT_POS0 (pRegData[0], BMI3_NO_MOTION_OUT, disable);
        pRegData[0] = BMI3_SET_BITS (pRegData[0], BMI3_ANY_MOTION_OUT, disable);
        pRegData[0] = BMI3_SET_BITS (pRegData[0], BMI3_FLAT_OUT, disable);
        pRegData[0] = BMI3_SET_BITS (pRegData[0], BMI3_ORIENTATION_OUT, disable);
        // NOLINTEND(*)

        temp = ((uint16_t)(pRegData[1])) << 8U;
        // NOLINTBEGIN(*)
        temp = BMI3_SET_BITS (temp, BMI3_STEP_DETECTOR_OUT, disable);
        temp = BMI3_SET_BITS (temp, BMI3_STEP_COUNTER_OUT, disable);
        temp = BMI3_SET_BITS (temp, BMI3_SIG_MOTION_OUT, disable);
        temp = BMI3_SET_BITS (temp, BMI3_TILT_OUT, disable);
        // NOLINTEND(*)
        pRegData[1] = (uint8_t)(temp >> 8U);
        // NOLINTBEGIN(*)
        pRegData[2] = BMI3_SET_BIT_POS0 (pRegData[2], BMI3_TAP_OUT, disable);
        pRegData[2] = BMI3_SET_BITS (pRegData[2], BMI3_I3C_OUT, disable);
        pRegData[2] = BMI3_SET_BITS (pRegData[2], BMI3_ERR_STATUS, disable);
        pRegData[2] = BMI3_SET_BITS (pRegData[2], BMI3_TEMP_DRDY_INT, enable);
        // NOLINTEND(*)
        temp = (uint16_t)(pRegData[3]) << 8U;
        // NOLINTBEGIN(*)
        temp = BMI3_SET_BITS (temp, BMI3_GYR_DRDY_INT, enable);
        temp = BMI3_SET_BITS (temp, BMI3_ACC_DRDY_INT, enable);
        temp = BMI3_SET_BITS (temp, BMI3_FIFO_WATERMARK_INT, disable);
        temp = BMI3_SET_BITS (temp, BMI3_FIFO_FULL_INT, disable);
        // NOLINTEND(*)
        pRegData[3] = (uint8_t)(temp >> 8U);

        status = IMUWriteReg (pIMU, BMI3_REG_INT_MAP1, pRegData, 4);
    }
    return status;
}

// STATIC eSTATUS_t IMUEnableInterrupts (vIMU_t const* pIMU) {

//     if (pIMU == NULL) {
//         return (eSTATUS_t)eIMU_NULL_PTR;
//     }

//     // Enable INT1 and INT2 with active high
//     uint8_t pEnableInterrupts[2] = { (1U << 2U | 1U << 0U), (1U << 2U | 1U << 0U) };
//     eSTATUS_t status             = IMUWriteReg (pIMU, BMI3_REG_IO_INT_CTRL, pEnableInterrupts, 2);

//     return status;
// }

STATIC eSTATUS_t IMUDisableInterrupts (vIMU_t* pIMU) {

    if (pIMU == NULL) {
        return (eSTATUS_t)eIMU_NULL_PTR;
    }

    // Enable INT1 and INT2 with active high
    uint8_t pDisableInterrupts[2] = { 0, 0 };
    eSTATUS_t status              = IMUWriteReg (pIMU, BMI3_REG_IO_INT_CTRL, pDisableInterrupts, 2);

    return status;
}

STATIC eSTATUS_t IMUConvertRaw (IMU_ACC_RANGE aRange, Vec3i ra, IMU_GYRO_RANGE gRange, Vec3i rg, Vec3f* pAccelOut, Vec3f* pGyroOut) {

    if (pAccelOut == NULL || pGyroOut == NULL) {
        return (eSTATUS_t)eIMU_NULL_PTR;
    }

    float scale = 2.0F;
    if (aRange == eIMU_ACC_RANGE_4G) {
        scale = 4.0F;
    } else if (aRange == eIMU_ACC_RANGE_8G) {
        scale = 8.0F;
    } else if (aRange == eIMU_ACC_RANGE_16G) {
        scale = 16.0F;
    }
    pAccelOut->x = ((float)ra.x * scale * 9.81F) / 32768.0F;
    pAccelOut->y = ((float)ra.y * scale * 9.81F) / 32768.0F;
    pAccelOut->z = ((float)ra.z * scale * 9.81F) / 32768.0F;

    scale = 125.0F;
    if (gRange == eIMU_GYRO_RANGE_250) {
        scale = 250.0F;
    } else if (gRange == eIMU_GYRO_RANGE_500) {
        scale = 500.0F;
    } else if (gRange == eIMU_GYRO_RANGE_1000) {
        scale = 1000.0F;
    } else if (gRange == eIMU_GYRO_RANGE_2000) {
        scale = 2000.0F;
    }
    pGyroOut->x = (scale / 32768.0F) * (float)rg.x;
    pGyroOut->y = (scale / 32768.0F) * (float)rg.y;
    pGyroOut->z = (scale / 32768.0F) * (float)rg.z;

    return eSTATUS_SUCCESS;
}


eSTATUS_t IMU_Init_ (IMUInitConf_t conf, vIMU_t* pOutIMU) {

    if (!pOutIMU) {
        LOG_ERROR ("Output vIMU_t pointer is NULL");
        return eSTATUS_FAILURE;
    }

    IMUAccConf accConf             = conf.aconf;
    IMUGyroConf gyroConf           = conf.gconf;
    IMUAxesRemapConf axesRemapConf = conf.axesRemapConf;

    eDEVICE_ID_t deviceId = eIMU_DEVICE_ID;

    eSTATUS_t status = eSTATUS_SUCCESS;

    IMU_t* pIMU = pOutIMU;
    memset (pIMU, 0, sizeof (vIMU_t));
    pIMU->deviceId = deviceId;
    pIMU->aconf    = accConf;
    pIMU->gconf    = gyroConf;

    pIMU->spiDev.cfg.busId    = IMU_SPI_BUS_ID;
    pIMU->spiDev.cfg.pNssPort = IMU_SPI_NSS_GPIO_PORT;
    pIMU->spiDev.cfg.nssPin   = IMU_SPI_NSS_GPIO_PIN;
    status                    = SpiDev_Init (&pIMU->spiDev);
    GOTO_IF (STATUS_FAIL (status), error, "Failed to setup spi device for imu");

    // if (pBusOverride != NULL) {
    //     pIMU->bus = *pBusOverride;
    // } else {
    //     BUS_INIT (&status, device, *pBus, &pIMU->bus);
    //     GOTO_IF (STATUS_FAIL (status), error, "Failed to init bus for imu");
    // }

    // if (pIMU->bus.ReadBlocking == NULL || pIMU->bus.WriteBlocking == NULL || pIMU->bus.WriteReadBlocking == NULL) {
    //     LOG_ERROR ("Bus for imu does not have read or write or write read function");
    //     goto error;
    // }

    /*
     * Soft reset vIMU_t and switch to SPI
     */
    status = IMUSoftReset (pIMU);
    GOTO_IF (STATUS_FAIL (status), error, "Failed to soft reset imu");
    LOG_INFO ("IMU soft reset successful");

    status = IMUSetAxesRemap (pIMU, axesRemapConf);
    GOTO_IF (STATUS_FAIL (status), error, "Failed to set imu axes remap");
    LOG_INFO ("Successfully remapped imu axes");

    /*
     * Setup the accel and gyro using the provided configurations
     */
    status = IMUSetConf (pIMU, &accConf, &gyroConf);
    GOTO_IF (STATUS_FAIL (status), error, "Failed to set imu config");
    LOG_INFO ("IMU configuration successful");
    IMU_LogDeviceConf (pIMU);

    uint8_t pChipID[2] = { 0 };
    status             = IMUReadReg (pIMU, BMI3_REG_CHIP_ID, pChipID, 2U);
    GOTO_IF (STATUS_FAIL (status), error, "Failed to read imu chip id");
    GOTO_IF (pChipID[0] != IMU_CHIP_ID, error, "Unexpected imu chip id");

    /* Self Calibrate */
    status = IMUCalibrate (pIMU, BMI3_SC_SENSITIVITY_EN | BMI3_SC_OFFSET_EN, BMI3_SC_APPLY_CORR_EN);
    GOTO_IF (STATUS_FAIL (status), error, "Failed to self calibrate IMU");
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
    // HAL_NVIC_SetPriority (IMU_INT_EXTI_IRQn, 8, 8);
    // HAL_NVIC_EnableIRQ (IMU_INT_EXTI_IRQn);

    // if (pExti != NULL) {
    //     /* Enable acc, gyro, and temperature - data ready interrupts for pin INT1 */
    //     status = IMUSetupInterrupts (pIMU);
    //     GOTO_IF (STATUS_FAIL (status), error, "Failed to setup imu interrupts");

    //     pIMU->usingEXTIInterrupt = true;
    // }

    pIMU->isInitialized = true;
    return eSTATUS_SUCCESS;
error:
    memset (pIMU, 0, sizeof (vIMU_t));
    return eSTATUS_FAILURE;
}

/*
 * Called by the interrupt handler
 */
STATIC UNUSED_FN_DECL bool IMUUpdatefromINT (vIMU_t* pIMU) {

    IMU_INTStatusReg_t INTStatus = { 0 };
    eSTATUS_t status             = IMUGetINTStatus (pIMU, &INTStatus);
    if (status != eSTATUS_SUCCESS) {
        return status;
    }

    if (INT_STATUS_ACCEL_DATA_READY (INTStatus)) {
        if (IMUUpdateRawAccel (pIMU) != eSTATUS_SUCCESS) {
            return eSTATUS_FAILURE;
        }
    }

    if (INT_STATUS_GYRO_DATA_READY (INTStatus)) {
        if (IMUUpdateRawGyro (pIMU) != eSTATUS_SUCCESS) {
            return eSTATUS_FAILURE;
        }
    }
    return eSTATUS_SUCCESS;
}

STATIC bool IMUUpdatefromPolling (vIMU_t* pIMU) {

    bool accelRdy    = false;
    bool gyroRdy     = false;
    int32_t timeout  = 1000;
    eSTATUS_t status = eSTATUS_SUCCESS;
    while (timeout-- > 0) {

        // IMU_INTStatusReg_t INTStatus = { 0 };
        // status             = IMUGetINTStatus (pIMU, &INTStatus);
        // RETURN_IF (STATUS_FAIL (status), false, "Failed to read vIMU_t interrupt status register");

        IMU_SysStatusReg_t SYSStatus = { 0 };
        status                       = IMUGetSysStatus (pIMU, &SYSStatus);
        RETURN_IF (STATUS_FAIL (status), false, "Failed to read vIMU_t status register");

        if (SYS_STATUS_ACCEL_DATA_READY (SYSStatus) && accelRdy == false) {
            status = IMUUpdateRawAccel (pIMU);
            RETURN_IF (STATUS_FAIL (status), false, "Failed to update accelerometer data");
            accelRdy = true;
        }

        if (SYS_STATUS_GYRO_DATA_READY (SYSStatus) && gyroRdy == false) {
            status = IMUUpdateRawGyro (pIMU);
            RETURN_IF (STATUS_FAIL (status), false, "Failed to update gyroscope data");
            gyroRdy = true;
        }
        if (accelRdy && gyroRdy) {
            break;
        }
        DelayMicroseconds (10);
        // HAL_Delay (1);
    }

    if (accelRdy == false) {
        LOG_ERROR ("IMU accelerometer data not ready after polling");
    }

    if (gyroRdy == false) {
        LOG_ERROR ("IMU gyroscope data not ready after polling");
    }

    return accelRdy && gyroRdy;
}

static eSTATUS_t IMUUpdate_ (vIMU_t* pIMU, bool forcePolling, Vec3f* pOutputAccel, Vec3f* pOutputGyro) {

    RETURN_IF (IMU_VALID (pIMU) == false, (eSTATUS_t)eIMU_NULL_PTR, "invalid imu pointer");
    RETURN_IF_NULL (pOutputAccel, (eSTATUS_t)eIMU_NULL_PTR, "output accel pointer is NULL");
    RETURN_IF_NULL (pOutputGyro, (eSTATUS_t)eIMU_NULL_PTR, "output gyro pointer is NULL");

    eSTATUS_t status = eSTATUS_SUCCESS;
    bool success     = true;
    bool usePolling  = pIMU->usingEXTIInterrupt == false || forcePolling == true;
    if (usePolling == true) {
        success = IMUUpdatefromPolling (pIMU);
    }

    success &= pIMU->gyroDataUpdated & pIMU->accelDataUpdated;
    if (success == true) {
        status =
        IMUConvertRaw (pIMU->aconf.range, pIMU->rawAccel, pIMU->gconf.range, pIMU->rawGyro, pOutputAccel, pOutputGyro);
        RETURN_IF (STATUS_FAIL (status), eSTATUS_FAILURE, "Failed to convert vIMU_t raw data");
        pIMU->gyroDataUpdated  = false;
        pIMU->accelDataUpdated = false;
    } else {

        /*
         * NOTE: for some reason the IMU will lose the ACCEL configuration after a few data reads.
         * Re-applying the configuration seems to fix the issue. Should only happen once.
         */
        status = IMUSetConf (pIMU, &pIMU->aconf, &pIMU->gconf);
        if (STATUS_OK (status)) {
            LOG_WARN ("IMU failed to update data so re-applying IMU configuration and retrying");
            return eSTATUS_RETRY;
        }

        LOG_ERROR ("IMU failed to update sensor data");
        IMU_LogError (pIMU);
        return eSTATUS_FAILURE;
    }
    return status;
}

eSTATUS_t IMU_Update (vIMU_t* pIMU, bool forcePolling, Vec3f* pOutputAccel, Vec3f* pOutputGyro) {

    eSTATUS_t status = IMUUpdate_ (pIMU, forcePolling, pOutputAccel, pOutputGyro);
    if (STATUS_RETRY (status)) {
        status = IMUUpdate_ (pIMU, forcePolling, pOutputAccel, pOutputGyro);
    }
    return status;
}

/*
 * Attempts to handle vIMU_t errors
 */
eSTATUS_t IMUHandleErr (vIMU_t* pIMU) {

    IMUErr err = { 0 };
    if (IMUGetDeviceErr (pIMU, &err) != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to read vIMU_t error codes");
        pIMU->status = eSTATUS_FAILURE;
        return eSTATUS_FAILURE;
    }

    IMU_LogDeviceErr (pIMU, &err);
    return eSTATUS_FAILURE;
}

void IMU2CPUInterruptHandler (vIMU_t* pIMU) {

    eSTATUS_t status = eSTATUS_SUCCESS;
    if (pIMU == NULL) {
        status = (eSTATUS_t)eIMU_NULL_PTR;
        goto error;
    }

    IMU_INTStatusReg_t INTStatus = { 0 };
    status                       = IMUGetINTStatus (pIMU, NULL);
    if (status != eSTATUS_SUCCESS) {
        goto error;
    }

    if (INT_STATUS_HAS_ERROR (INTStatus)) {
        status = (eSTATUS_t)eIMU_HARDWARE_ERR;
        goto error;
    }

    if (INT_STATUS_ACCEL_DATA_READY (INTStatus)) {
        status = IMUUpdateRawAccel (pIMU);
        if (status != eSTATUS_SUCCESS) {
            goto error;
        }
    }

    if (INT_STATUS_GYRO_DATA_READY (INTStatus)) {
        status = IMUUpdateRawGyro (pIMU);
        if (status != eSTATUS_SUCCESS) {
            goto error;
        }
    }

    /* check if temperature data is ready */
    if (INT_STATUS_TEMP_DATA_READY (INTStatus)) {
        // if (status != eSTATUS_SUCCESS) {
        //     goto error;
        // }
    };

error:
    pIMU->status = status;
}

eSTATUS_t IMUGetConf (vIMU_t* pIMU, IMUAccConf* pAConf, IMUGyroConf* pGConf) {
    return IMUGetConf_ (pIMU, pAConf, pGConf, false);
}

eSTATUS_t IMUGetAltConf (vIMU_t* pIMU, IMUAccConf* pAConf, IMUGyroConf* pGConf) {
    return IMUGetConf_ (pIMU, pAConf, pGConf, true);
}

eSTATUS_t IMUSetConf (vIMU_t* pIMU, IMUAccConf const* pAConf, IMUGyroConf const* pGConf) {
    return IMUSetConf_ (pIMU, pAConf, pGConf, false);
}

eSTATUS_t IMUSetAltConf (vIMU_t* pIMU, IMUAccConf const* pAConf, IMUGyroConf const* pGConf) {
    return IMUSetConf_ (pIMU, pAConf, pGConf, true);
}

eSTATUS_t IMUCompareConfs (IMUAccConf aconf, IMUGyroConf gconf, IMUAccConf aconf2, IMUGyroConf gconf2) {

    if (aconf.mode != aconf2.mode || aconf.odr != aconf2.odr || aconf.range != aconf2.range ||
        aconf.avg != aconf2.avg || aconf.bw != aconf2.bw) {
        return eSTATUS_FAILURE;
    }
    if (gconf.mode != gconf2.mode || gconf.odr != gconf2.odr || gconf.range != gconf2.range ||
        gconf.avg != gconf2.avg || gconf.bw != gconf2.bw) {
        return eSTATUS_FAILURE;
    }
    return eSTATUS_SUCCESS;
}

vIMU_t const* IMUGetActiveDevice (void) {

    if (IMU_VALID (&g_IMU) == false) {
        // LOG_ERROR ("No active valid IMU device");
        return NULL;
    }
    return &g_IMU;
}

vIMU_t* IMU_GetMutableActiveDevice (void) {

    if (IMU_VALID (&g_IMU) == false) {
        // LOG_ERROR ("No active valid IMU device");
        return NULL;
    }
    return &g_IMU;
}
