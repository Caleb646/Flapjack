#include "sensors/imu/imu.h"
#include "common.h"
#include "conf.h"
#include "log.h"
#include "periphs/gpio.h"
#include "sensors/imu/bmixxx.h"
#include <stdint.h>
#include <string.h>

#define BIT_ISSET(v, bit) (((v) & (bit)) > 0U)
#define CHECK_INT_ERR_STATUS(u16IntStatus) \
    BIT_ISSET (u16IntStatus, (1U << 10U))

enum {
    RW_BUFFER_SZ              = 16U,
    INT_ERR_STATUS_BIT        = 10U,
    SPI_DEFAULT_TIMEOUT_MS    = 100U,
    INT1_ACCEL_DATA_RDY_BIT   = (1U << 13U),
    INT1_GYRO_DATA_RDY_BIT    = (1U << 12U),
    INT1_TEMP_DATA_RDY_BIT    = (1U << 11U),
    STATUS_ACCEL_DATA_RDY_BIT = (1U << 7U),
    STATUS_GYRO_DATA_RDY_BIT  = (1U << 6U),
    STATUS_TEMP_DATA_RDY_BIT  = (1U << 5U)
};

// IMU gIMU = { 0 };

#ifndef UNIT_TEST

static eSTATUS_t IMUSendCmd (IMU const* pIMU, uint16_t cmd);
static eSTATUS_t
IMUGetFeatureStatus (IMU const* pIMU, uint16_t featureRegAddr, IMUFeatureStatus* pResultOut);
static eSTATUS_t IMUGetINTStatus (IMU const* pIMU, uint16_t* pOutStatus);
static eSTATUS_t IMUGetStatusReg (IMU const* pIMU, uint16_t* pOutStatus);
static eSTATUS_t IMUGetDeviceErr (IMU* pIMU, IMUErr* pOutErr);
static void IMULogDeviceErr (IMU* pIMU, IMUErr const* pErr);
static eSTATUS_t IMUReadReg (IMU const* pIMU, uint8_t reg, uint8_t* pBuf, uint32_t len);
static eSTATUS_t IMUWriteReg (IMU const* pIMU, uint8_t reg, uint8_t* pBuf, uint32_t len);
static eSTATUS_t IMUUpdateGyro (IMU* pIMU);
static eSTATUS_t IMUUpdateAccel (IMU* pIMU);
static eSTATUS_t IMUSetAxesRemap (IMU* pIMU, IMUAxesRemapConf remap);
static eSTATUS_t IMUSoftReset (IMU* pIMU);
static eSTATUS_t
IMUGetConf_ (IMU* pIMU, IMUAccConf* pAConf, IMUGyroConf* pGConf, uint8_t altConfFlag);
static eSTATUS_t
IMUSetConf_ (IMU* pIMU, IMUAccConf const* pAConf, IMUGyroConf const* pGConf, uint8_t altConfFlag);
static eSTATUS_t
IMUCalibrate (IMU* pIMU, uint8_t calibSelection, uint8_t applyCorrection, IMUSelfCalibResult* pResultOut);
static eSTATUS_t IMUSetupInterrupts (IMU const* pIMU);
static eSTATUS_t IMUEnableInterrupts (IMU const* pIMU);
static eSTATUS_t IMUDisableInterrupts (IMU const* pIMU);
static eSTATUS_t
IMUConvertRaw (IMU_ACC_RANGE aRange, Vec3 ra, IMU_GYRO_RANGE gRange, Vec3 rg, Vec3f* pAccelOut, Vec3f* pGyroOut);

#endif

STATIC_TESTABLE_DECL eSTATUS_t IMUSendCmd (IMU const* pIMU, uint16_t cmd) {
    uint8_t pRegData[2] = { 0 };
    pRegData[0]         = (uint8_t)(cmd & BMI3_SET_LOW_BYTE);
    pRegData[1]         = (uint8_t)((cmd & BMI3_SET_HIGH_BYTE) >> 8U);
    eSTATUS_t status    = IMUWriteReg (pIMU, BMI3_REG_CMD, pRegData, 2);
    if (status != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to send IMU command 0x%04X", cmd);
        return status;
    }
    return eSTATUS_SUCCESS;
}

STATIC_TESTABLE_DECL eSTATUS_t
IMUGetFeatureStatus (IMU const* pIMU, uint16_t featureRegAddr, IMUFeatureStatus* pResultOut) {
    uint8_t pData[2] = { 0 };
    eSTATUS_t status = IMUReadReg (pIMU, featureRegAddr, pData, 2);
    if (status != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to read IMU feature status register [0x%04X]", featureRegAddr);
        return status;
    }

    uint16_t featureIO = ((uint16_t)pData[1] << 8U) | (uint16_t)pData[0];
    if (featureRegAddr == BMI3_REG_FEATURE_IO1) {
        pResultOut->errStatus           = featureIO & (0xFU << 0U);
        pResultOut->selfCalibComplete   = (featureIO & (1U << 4U)) > 0;
        pResultOut->gyroSelfCalibResult = (featureIO & (1U << 5U)) > 0;
        pResultOut->selfTestResult      = (featureIO & (1U << 6U)) > 0;
        pResultOut->axisRemapComplete   = (featureIO & (1U << 7U)) > 0;
        pResultOut->systemState         = (featureIO & (3U << 11U)) >> 11U;
        return eSTATUS_SUCCESS;
    }
    LOG_ERROR ("Trying read from unsupported feature status register [0x%04X]", featureRegAddr);
    return eSTATUS_FAILURE;
}

STATIC_TESTABLE_DECL eSTATUS_t IMUGetINTStatus (IMU const* pIMU, uint16_t* pOutStatus) {
    uint8_t pBuff[2] = { 0U };
    eSTATUS_t status = IMUReadReg (pIMU, BMI3_REG_INT_STATUS_INT1, pBuff, 2);
    if (status != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to read IMU interrupt status register");
        return status;
    }
    *pOutStatus = ((uint16_t)pBuff[1] << 8U) | (uint16_t)pBuff[0];
    return eSTATUS_SUCCESS;
}

STATIC_TESTABLE_DECL eSTATUS_t IMUGetStatusReg (IMU const* pIMU, uint16_t* pOutStatus) {
    uint8_t pBuff[2] = { 0U };
    eSTATUS_t status = IMUReadReg (pIMU, BMI3_REG_STATUS, pBuff, 2);
    if (status != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to read IMU status register");
        return status;
    }
    *pOutStatus = ((uint16_t)pBuff[1] << 8U) | (uint16_t)pBuff[0];
    return eSTATUS_SUCCESS;
}

STATIC_TESTABLE_DECL eSTATUS_t IMUGetDeviceErr (IMU* pIMU, IMUErr* pOutErr) {
    uint8_t pBuff[2]  = { 0U };
    eSTATUS_t status  = IMUReadReg (pIMU, BMI3_REG_ERR_REG, pBuff, 2);
    uint16_t err      = ((uint16_t)pBuff[1] << 8U) | (uint16_t)pBuff[0];
    pOutErr->err      = err;
    pOutErr->fatalErr = (err & (1U << 0U)) > 0;
    pOutErr->featEngOvrld = (err & (1U << 2U)) > 0;
    pOutErr->featEngWd    = (err & (1U << 4U)) > 0;
    pOutErr->accConfErr   = (err & (1U << 5U)) > 0;
    pOutErr->gyrConfErr   = (err & (1U << 6U)) > 0;
    pOutErr->i3cErr0      = (err & (1U << 8U)) > 0;
    pOutErr->i3cErr1      = (err & (1U << 11U)) > 0;
    if (status != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to read IMU error register");
        return status;
    }
    return eSTATUS_SUCCESS;
}

STATIC_TESTABLE_DECL void IMULogDeviceErr (IMU* pIMU, IMUErr const* pErr) {
    IMUErr err;
    if (pErr == NULL) {
        if (IMUGetDeviceErr (pIMU, &err) != eSTATUS_SUCCESS) {
            LOG_ERROR ("Failed to read IMU error codes");
            return;
        }
    } else {
        err = *pErr;
    }

    if (err.fatalErr != 0) {
        LOG_ERROR (
        "IMU fatal error, chip is not in operation state (Boot or "
        "Power-System). Power on reset or soft reset required");
    }
    if (err.featEngOvrld != 0) {
        LOG_ERROR ("IMU overload of the feature engine detected");
    }
    if (err.featEngWd != 0) {
        LOG_ERROR ("IMU watchdog timer of the feature engine triggered");
    }
    if (err.accConfErr != 0) {
        LOG_ERROR ("IMU unsupported accelerometer configuration set by user");
    }
    if (err.gyrConfErr != 0) {
        LOG_ERROR ("IMU unsupported gyroscope configuration set by user");
    }
    if (err.i3cErr0 != 0) {
        LOG_ERROR ("IMU I3C SDR parity error occurred");
    }
    if (err.i3cErr1 != 0) {
        LOG_ERROR ("IMU I3C S0/S1 error occurred");
    } else {
        LOG_ERROR ("Did NOT find any IMU device errors");
    }
}

STATIC_TESTABLE_DECL eSTATUS_t IMUReadReg (IMU const* pIMU, uint8_t reg, uint8_t* pBuf, uint32_t len) {

    uint8_t pTx[RW_BUFFER_SZ] = { 0 };
    // set read mask for register address
    pTx[0] = BMI3_SPI_RD_MASK | reg;

    uint8_t pRx[RW_BUFFER_SZ] = { 0 };
    /*
     * NOTE: Position        Sent        Received
     *       0               addr         0x00
     *       1               0x00         dummy byte (0x00)
     *       2               0x00         data byte 0
     *       N               0x00         data byte N - 1
     */
    if (len + pIMU->nDummyBytes + 1 > RW_BUFFER_SZ) {
        return (eSTATUS_t)eIMU_RW_BUFFER_OVERFLOW;
    }

    if (HAL_SPI_TransmitReceive (pIMU->pSPI, pTx, pRx, len + pIMU->nDummyBytes + 1, SPI_DEFAULT_TIMEOUT_MS) != HAL_OK) {
        return (eSTATUS_t)eIMU_COM_FAILURE;
    }

    // Add 2 microsecond delay after IMU read operation
    DelayMicroseconds (2);

    // The first nDummyBytes + 1 (for the register address) are dummy bytes
    memcpy (pBuf, &(pRx[pIMU->nDummyBytes + 1]), len);
    return eSTATUS_SUCCESS;
}

STATIC_TESTABLE_DECL eSTATUS_t IMUWriteReg (IMU const* pIMU, uint8_t reg, uint8_t* pBuf, uint32_t len) {

    uint8_t pTx[RW_BUFFER_SZ] = { 0 };
    if (len + 1 > RW_BUFFER_SZ) {
        return (eSTATUS_t)eIMU_RW_BUFFER_OVERFLOW;
    }

    pTx[0] = reg & BMI3_SPI_WR_MASK;
    memcpy (&pTx[1], (void*)pBuf, len);

    if (HAL_SPI_Transmit (pIMU->pSPI, pTx, len + 1, SPI_DEFAULT_TIMEOUT_MS) != HAL_OK) {
        return (eSTATUS_t)eIMU_COM_FAILURE;
    }

    // Add 2 microsecond delay after IMU write operation
    DelayMicroseconds (2);

    return eSTATUS_SUCCESS;
}

STATIC_TESTABLE_DECL eSTATUS_t IMUUpdateGyro (IMU* pIMU) {

    uint8_t pBuffer[6] = { 0 };
    eSTATUS_t status = IMUReadReg (pIMU, BMI3_REG_GYR_DATA_X, pBuffer, 6);
    if (status != eSTATUS_SUCCESS) {
        return status;
    }

    pIMU->rawGyro.x =
    (int16_t)((((uint16_t)pBuffer[1]) << 8U) | ((uint16_t)pBuffer[0]));
    pIMU->rawGyro.y =
    (int16_t)((((uint16_t)pBuffer[3]) << 8U) | ((uint16_t)pBuffer[2]));
    pIMU->rawGyro.z =
    (int16_t)((((uint16_t)pBuffer[5]) << 8U) | ((uint16_t)pBuffer[4]));

    return eSTATUS_SUCCESS;
}

STATIC_TESTABLE_DECL eSTATUS_t IMUUpdateAccel (IMU* pIMU) { // , Vec3 curVel, Vec3* pOutputAccel) {
    uint8_t pBuffer[6] = { 0 };
    eSTATUS_t status = IMUReadReg (pIMU, BMI3_REG_ACC_DATA_X, pBuffer, 6);
    if (status != eSTATUS_SUCCESS) {
        return status;
    }

    pIMU->rawAccel.x =
    (int16_t)((((uint16_t)pBuffer[1]) << 8U) | ((uint16_t)pBuffer[0]));
    pIMU->rawAccel.y =
    (int16_t)((((uint16_t)pBuffer[3]) << 8U) | ((uint16_t)pBuffer[2]));
    pIMU->rawAccel.z =
    (int16_t)((((uint16_t)pBuffer[5]) << 8U) | ((uint16_t)pBuffer[4]));

    return eSTATUS_SUCCESS;
}

STATIC_TESTABLE_DECL eSTATUS_t IMUSetAxesRemap (IMU* pIMU, IMUAxesRemapConf remap) {

    uint8_t addr[2] = { BMI3_BASE_ADDR_AXIS_REMAP, 0 };
    uint8_t data    = 0;

    /* Set the configuration to feature engine register */
    eSTATUS_t status = IMUWriteReg (pIMU, BMI3_REG_FEATURE_DATA_ADDR, addr, 2);
    if (status != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to set IMU feature data address for axis remap");
        return status;
    }

    data = BMI3_SET_BIT_POS0 (data, BMI3_XYZ_AXIS, remap.remap);
    data |= BMI3_SET_BITS (data, BMI3_X_AXIS_SIGN, remap.xDir);
    data |= BMI3_SET_BITS (data, BMI3_Y_AXIS_SIGN, remap.yDir);
    data |= BMI3_SET_BITS (data, BMI3_Z_AXIS_SIGN, remap.zDir);
    uint8_t aSend[2] = { data, 0 };

    status = IMUWriteReg (pIMU, BMI3_REG_FEATURE_DATA_TX, aSend, 2);
    if (status != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to set IMU feature data TX for axis remap");
        return status;
    }

    /*
     * NOTE: The command to start the axis remap update can be sent without
     * checking the enabled/disabled status of the accel because this
     * function is only called after an IMU soft reset.
     */
    status = IMUSendCmd (pIMU, BMI3_CMD_AXIS_MAP_UPDATE);
    if (status != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to send IMU command to update axis remap");
        return status;
    }

    int16_t wait = 1000;
    status       = eSTATUS_FAILURE;
    while (wait-- > 0) {
        IMUFeatureStatus featStatus = { 0 };
        status = IMUGetFeatureStatus (pIMU, BMI3_REG_FEATURE_IO1, &featStatus);
        if (status != eSTATUS_SUCCESS) {
            LOG_ERROR ("Failed to get IMU feature status");
            return status;
        }
        if ((featStatus.errStatus & BMI3_NO_ERROR_MASK) && featStatus.axisRemapComplete > 0) {
            LOG_INFO ("IMU axis remap complete");
            status = eSTATUS_SUCCESS;
            break;
        }
        HAL_Delay (1);
    }

    if (status != eSTATUS_SUCCESS) {
        LOG_ERROR ("IMU axis remap did not complete in time");
        IMULogDeviceErr (pIMU, NULL);
        return status;
    }

    pIMU->axesRemapConf.remap = remap.remap;
    return eSTATUS_SUCCESS;
}

STATIC_TESTABLE_DECL eSTATUS_t IMUSoftReset (IMU* pIMU) {
    /* Send soft reset command to BMI323 */
    eSTATUS_t status = IMUSendCmd (pIMU, BMI3_CMD_SOFT_RESET);

    if (status != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to send soft reset command to IMU");
        return status;
    }

    HAL_Delay (100);
    /* Perform dummy read to switch from I3C/I2C to SPI */
    if (status == eSTATUS_SUCCESS) {
        uint8_t dummyBytes[2] = { 0 };
        status = IMUReadReg (pIMU, BMI3_REG_CHIP_ID, dummyBytes, 2);
    }

    HAL_Delay (100);
    /* Enable feature engine */
    if (status == eSTATUS_SUCCESS) {
        uint8_t featureData[2] = { 0x2C, 0x01 };
        status = IMUWriteReg (pIMU, BMI3_REG_FEATURE_IO2, featureData, 2);
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
        status = IMUWriteReg (pIMU, BMI3_REG_FEATURE_CTRL, featureEngine, 2);
    }

    uint8_t featEnabled = FALSE;
    if (status == eSTATUS_SUCCESS) {
        int16_t loop       = 0;
        uint8_t regData[2] = { 0 };

        while (loop++ <= 20) {
            HAL_Delay (100);
            status = IMUReadReg (pIMU, BMI3_REG_FEATURE_IO1, regData, 2);
            if (status == eSTATUS_SUCCESS) {
                if (regData[0] & (uint16_t)BMI3_FEATURE_ENGINE_ENABLE_MASK) {
                    featEnabled = TRUE;
                    break;
                }
            }
        }
    }
    if (featEnabled != TRUE) {
        LOG_ERROR ("Failed to enable feature engine after soft reset");
        IMULogDeviceErr (pIMU, NULL);
        return eSTATUS_FAILURE;
    }
    return status;
}

STATIC_TESTABLE_DECL eSTATUS_t
IMUGetConf_ (IMU* pIMU, IMUAccConf* pAConf, IMUGyroConf* pGConf, uint8_t altConfFlag) {
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

STATIC_TESTABLE_DECL eSTATUS_t
IMUSetConf_ (IMU* pIMU, IMUAccConf const* pAConf, IMUGyroConf const* pGConf, uint8_t altConfFlag) {
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
        odr   = BMI3_SET_BIT_POS0 (pRegData[0], BMI3_ACC_ODR, pAConf->odr);
        range = BMI3_SET_BITS (pRegData[0], BMI3_ACC_RANGE, pAConf->range);
        bwp   = BMI3_SET_BITS (pRegData[0], BMI3_ACC_BW, pAConf->bw);
        avgNum = BMI3_SET_BITS (pRegData[1], BMI3_ACC_AVG_NUM, pAConf->avg);
        accMode = BMI3_SET_BITS (pRegData[1], BMI3_ACC_MODE, pAConf->mode);

        if (altConfFlag == TRUE) {
            regAddr = BMI3_REG_ALT_ACC_CONF;
            odr = BMI3_SET_BIT_POS0 (pRegData[0], BMI3_ALT_ACC_ODR, pAConf->odr);
            avgNum =
            BMI3_SET_BITS (pRegData[1], BMI3_ALT_ACC_AVG_NUM, pAConf->avg);
            accMode = BMI3_SET_BITS (pRegData[1], BMI3_ALT_ACC_MODE, pAConf->mode);
        }

        pRegData[0] = (uint8_t)(odr | range | bwp);
        pRegData[1] = (uint8_t)((avgNum | accMode) >> 8U);
        status      = IMUWriteReg (pIMU, regAddr, pRegData, 2);
        // NOLINTEND(*)
        if (status != eSTATUS_SUCCESS) {
            // LOG_ERROR ("Failed to configure IMU accelerometer");
            return status;
        }
        if (status == eSTATUS_SUCCESS && altConfFlag == 0) {
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
        odr   = BMI3_SET_BIT_POS0 (pRegData[0], BMI3_GYR_ODR, pGConf->odr);
        range = BMI3_SET_BITS (pRegData[0], BMI3_GYR_RANGE, pGConf->range);
        bwp   = BMI3_SET_BITS (pRegData[0], BMI3_GYR_BW, pGConf->bw);
        avgNum = BMI3_SET_BITS (pRegData[1], BMI3_GYR_AVG_NUM, pGConf->avg);
        accMode = BMI3_SET_BITS (pRegData[1], BMI3_GYR_MODE, pGConf->mode);

        if (altConfFlag == TRUE) {
            regAddr = BMI3_REG_ALT_GYR_CONF;
            odr = BMI3_SET_BIT_POS0 (pRegData[0], BMI3_ALT_GYR_ODR, pGConf->odr);
            avgNum =
            BMI3_SET_BITS (pRegData[1], BMI3_ALT_GYR_AVG_NUM, pGConf->avg);
            accMode = BMI3_SET_BITS (pRegData[1], BMI3_ALT_GYR_MODE, pGConf->mode);
        }

        pRegData[0] = (uint8_t)(odr | range | bwp);
        pRegData[1] = (uint8_t)((avgNum | accMode) >> 8U);
        status      = IMUWriteReg (pIMU, regAddr, pRegData, 2);
        // NOLINTEND(*)
        if (status != eSTATUS_SUCCESS) {
            LOG_ERROR ("Failed to configure IMU gyroscope");
            return eSTATUS_FAILURE;
        }
        if (status == eSTATUS_SUCCESS && altConfFlag == 0) {
            pIMU->gconf = *pGConf;
        }
    }
    return status;
}

STATIC_TESTABLE_DECL eSTATUS_t
IMUCalibrate (IMU* pIMU, uint8_t calibSelection, uint8_t applyCorrection, IMUSelfCalibResult* pResultOut) {

    if (pIMU == NULL || pResultOut == NULL) {
        LOG_ERROR ("IMU or result pointer is NULL");
        return (eSTATUS_t)eIMU_NULL_PTR;
    }

    pResultOut->result = FALSE;

    /* Save the current configs */
    IMUAccConf aconf;
    IMUGyroConf gconf;
    eSTATUS_t status = IMUGetConf (pIMU, &aconf, &gconf);
    if (status != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to get IMU configuration to save before calibration");
        return status;
    }
    HAL_Delay (100);

    /* Set the ACC config to be what the self calibration expects */
    IMUAccConf calibAConf = { 0 };
    calibAConf.mode       = eIMU_ACC_MODE_HIGH_PERF;
    calibAConf.odr        = eIMU_ACC_ODR_100;
    calibAConf.range      = aconf.range;
    calibAConf.avg        = aconf.avg;
    calibAConf.bw         = aconf.bw;
    status                = IMUSetConf (pIMU, &calibAConf, NULL);
    if (status != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to set IMU accelerometer configuration for calibration");
        return status;
    }
    HAL_Delay (100);

    /* Store alt configs and then disable them */
    IMUAccConf altAConf;
    IMUGyroConf altGConf;
    status = IMUGetAltConf (pIMU, &altAConf, &altGConf);
    if (status != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to get IMU alternate configuration to save before calibration");
        return status;
    }

    altAConf.mode = eIMU_ACC_MODE_DISABLE;
    altGConf.mode = eIMU_GYRO_MODE_DISABLE;
    status        = IMUSetAltConf (pIMU, &altAConf, &altGConf);
    if (status != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to disable IMU alternate configuration before calibration");
        return status;
    }
    HAL_Delay (100);

    /* Trigger the self calibration */
    status = IMUSendCmd (pIMU, BMI3_CMD_SELF_CALIB_TRIGGER);
    if (status != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to trigger IMU self-calibration");
        return status;
    }
    HAL_Delay (100);

    /* Check that the self calibration has started */
    {
        IMUFeatureStatus featureStatus = { 0 };
        status = IMUGetFeatureStatus (pIMU, BMI3_REG_FEATURE_IO1, &featureStatus);
        if (status != eSTATUS_SUCCESS) {
            LOG_ERROR ("Failed to get IMU feature status");
            return status;
        }
        if (featureStatus.systemState != 0x1U) {
            LOG_ERROR (
            "IMU has not started the self calibration. System state "
            "[0x%X]",
            featureStatus.systemState);
            IMULogDeviceErr (pIMU, NULL);
            return eSTATUS_FAILURE;
        }
    }

    /* Get self calibration result */
    {
        for (uint8_t idx = 0; idx < 10U; idx++) {
            /* A delay of 1000ms (100ms * 10(limit)) is required to perform self calibration */
            HAL_Delay (100);
            uint16_t intStatus = 0;
            status             = IMUGetINTStatus (pIMU, &intStatus);
            if (status != eSTATUS_SUCCESS) {
                LOG_ERROR ("Failed to read IMU interrupt status");
                return status;
            }
            if (CHECK_INT_ERR_STATUS (intStatus)) {
                break;
            }
        }

        IMUFeatureStatus featureStatus = { 0 };
        status = IMUGetFeatureStatus (pIMU, BMI3_REG_FEATURE_IO1, &featureStatus);
        if (status != eSTATUS_SUCCESS) {
            LOG_ERROR ("Failed to get IMU feature status");
            return status;
        }
        uint16_t scErrStatus = featureStatus.errStatus;
        uint16_t scComplete  = featureStatus.selfCalibComplete;
        uint16_t scResult    = featureStatus.gyroSelfCalibResult;
        uint16_t systemState = featureStatus.systemState;

        if (scComplete > 0 && scResult > 0 && scErrStatus == 0x5U && systemState == 0x00) {
            pResultOut->result = TRUE;
            pResultOut->error  = 0;
            return status;
        }

        pResultOut->result = FALSE;
        pResultOut->error  = (uint8_t)scErrStatus;
        LOG_ERROR ("IMU self-calibration failed. SC Err Status [0x%X] System State [0x%X]", scErrStatus, systemState);
        IMULogDeviceErr (pIMU, NULL);
    }
    HAL_Delay (20);
    /* Restore configs */
    status = IMUSetConf (pIMU, &aconf, &gconf);
    HAL_Delay (100);
    return status;
}

STATIC_TESTABLE_DECL eSTATUS_t IMUSetupInterrupts (IMU const* pIMU) {
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

STATIC_TESTABLE_DECL eSTATUS_t IMUEnableInterrupts (IMU const* pIMU) {
    if (pIMU == NULL || pIMU->pSPI == NULL) {
        return (eSTATUS_t)eIMU_NULL_PTR;
    }

    // Enable INT1 and INT2 with active high
    uint8_t pEnableInterrupts[2] = { (1U << 2U | 1U << 0U), (1U << 2U | 1U << 0U) };
    eSTATUS_t status =
    IMUWriteReg (pIMU, BMI3_REG_IO_INT_CTRL, pEnableInterrupts, 2);

    return status;
}

STATIC_TESTABLE_DECL eSTATUS_t IMUDisableInterrupts (IMU const* pIMU) {
    if (pIMU == NULL || pIMU->pSPI == NULL) {
        return (eSTATUS_t)eIMU_NULL_PTR;
    }

    // Enable INT1 and INT2 with active high
    uint8_t pDisableInterrupts[2] = { 0, 0 };
    eSTATUS_t status =
    IMUWriteReg (pIMU, BMI3_REG_IO_INT_CTRL, pDisableInterrupts, 2);

    return status;
}

STATIC_TESTABLE_DECL eSTATUS_t
IMUConvertRaw (IMU_ACC_RANGE aRange, Vec3 ra, IMU_GYRO_RANGE gRange, Vec3 rg, Vec3f* pAccelOut, Vec3f* pGyroOut) {

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


eSTATUS_t IMUInit (IMU* pIMU, SPI_HandleTypeDef* pSPI, IMUAxesRemapConf* pAxesRemapConf) {

    if (pIMU == NULL || pSPI == NULL) {
        LOG_ERROR ("IMU or SPI pointer is NULL");
        return (eSTATUS_t)eIMU_NULL_PTR;
    }

    memset (pIMU, 0, sizeof (IMU));
    pIMU->pSPI                 = pSPI;
    pIMU->msLastAccUpdateTime  = 0;
    pIMU->msLastGyroUpdateTime = 0;
    /* SPI reads have 1 dummy byte at the beginning */
    pIMU->nDummyBytes = 1;

    IMUAccConf aconf  = { 0 };
    aconf.odr         = eIMU_ACC_ODR_100;
    aconf.range       = eIMU_ACC_RANGE_2G;
    aconf.avg         = eIMU_ACC_AVG_16;
    aconf.bw          = eIMU_ACC_BW_HALF;
    aconf.mode        = eIMU_ACC_MODE_HIGH_PERF;
    IMUGyroConf gconf = { 0 };
    gconf.odr         = eIMU_GYRO_ODR_100;
    gconf.range       = eIMU_GYRO_RANGE_250;
    gconf.avg         = eIMU_GYRO_AVG_16;
    gconf.bw          = eIMU_GYRO_BW_HALF;
    gconf.mode        = eIMU_GYRO_MODE_HIGH_PERF;

    if (HZ_SENSOR_UPDATE_RATE >= 200U) {
        aconf.odr = eIMU_ACC_ODR_200;
        gconf.odr = eIMU_GYRO_ODR_200;
    }
    if (HZ_SENSOR_UPDATE_RATE >= 400U) {
        aconf.odr = eIMU_ACC_ODR_400;
        gconf.odr = eIMU_GYRO_ODR_400;
    }
    if (HZ_SENSOR_UPDATE_RATE >= 800U && HZ_SENSOR_UPDATE_RATE <= 1000U) {
        aconf.odr = eIMU_ACC_ODR_800;
        gconf.odr = eIMU_GYRO_ODR_800;
    } else {
        LOG_ERROR ("IMU update rate [%d] is not supported", (uint16_t)HZ_SENSOR_UPDATE_RATE);
    }

    /*
     * Soft reset IMU and switch to SPI
     */
    eSTATUS_t status = IMUSoftReset (pIMU);
    if (status != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to soft reset IMU");
        return status;
    }

    if (pAxesRemapConf != NULL) {
        status = IMUSetAxesRemap (pIMU, *pAxesRemapConf);
        if (status != eSTATUS_SUCCESS) {
            LOG_ERROR ("Failed to set IMU axes remap");
            return status;
        }
        LOG_INFO ("Successfully remapped IMU axes");
    }

    /*
     * Setup the accel and gyro using the provided configurations
     */
    status = IMUSetConf (pIMU, &aconf, &gconf);
    if (status != eSTATUS_SUCCESS) {
        LOG_ERROR ("IMU failed to configure accel or gyro");
        return status;
    }

    uint8_t pChipID[2] = { 0 };
    status             = IMUReadReg (pIMU, BMI3_REG_CHIP_ID, pChipID, 2);
    if (pChipID[0] != BMI323_CHIP_ID) {
        LOG_ERROR ("Chip ID [0x%X] [0x%X] is incorrect", pChipID[0], pChipID[1]);
        return eSTATUS_FAILURE;
    }

    /* Enable acc, gyro, and temperature - data ready interrupts for pin INT1 */
    status = IMUSetupInterrupts (pIMU);
    if (status != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to setup IMU interrupts");
        return eSTATUS_FAILURE;
    }

    /* Self Calibrate */
    {
        IMUSelfCalibResult calibResult = { 0 };
        status =
        IMUCalibrate (pIMU, BMI3_SC_SENSITIVITY_EN | BMI3_SC_OFFSET_EN, BMI3_SC_APPLY_CORR_EN, &calibResult);
        if (status != eSTATUS_SUCCESS) {
            LOG_ERROR ("Failed to self calibrate IMU");
            return status;
        }
        if (
        calibResult.result != TRUE ||
        calibResult.error !=
        ((calibResult.error & BMI3_SET_LOW_NIBBLE) == BMI3_NO_ERROR_MASK)) {
            LOG_ERROR (
            "Self calibration result: [%d], error: [0x%02X]",
            calibResult.result, calibResult.error);
            return eSTATUS_FAILURE;
        }
        LOG_INFO ("Self calibration result: [%d]", calibResult.result);
    }

    /* NOTE: For some reason the ODR value used during calibration is always
     * read back from the IMU. So a calibration odr of 100Hz and an user set odr of 200 Hz
     * causes the conf comparison to fail even after the user conf is restored.
     * But the 200 Hz odr rate is honored by the IMU. */
    /* Double check that the given confs stuck */
    {
        // HAL_Delay (100);
        // IMUAccConf aconf2;
        // IMUGyroConf gconf2;
        // status = IMUGetConf (pIMU, &aconf2, &gconf2);
        // if (status != eSTATUS_SUCCESS) {
        //     LOG_ERROR ("Failed to read back IMU configuration");
        //     IMULogDeviceErr (pIMU, NULL);
        //     return status;
        // }

        // status = IMUCompareConfs (aconf, gconf, aconf2, gconf2);
        // if (status != eSTATUS_SUCCESS) {
        //     LOG_ERROR (
        //     "IMU configuration mismatch after setting. "
        //     "Expected: Accel [%d %d %d %d] Gyro [%d %d %d %d] "
        //     "Got: Accel [%d %d %d %d] Gyro [%d %d %d %d]",
        //     aconf.mode, aconf.odr, aconf.range, aconf.avg, gconf.mode,
        //     gconf.odr, gconf.range, gconf.avg, aconf2.mode, aconf2.odr,
        //     aconf2.range, aconf2.avg, gconf2.mode, gconf2.odr,
        //     gconf2.range, gconf2.avg);
        //     IMULogDeviceErr (pIMU, NULL);
        //     return status;
        // }
    }

#ifndef UNIT_TEST
    /*
     * Setup GPIO for IMU data ready interrupt
     */
    IMU_INT_RCC_GPIO_CLK_ENABLE ();
    GPIO_InitTypeDef GPIO_InitStruct = { 0 };
    GPIO_InitStruct.Pin              = IMU_INT_GPIO_Pin;
    GPIO_InitStruct.Speed            = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Mode             = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull             = GPIO_NOPULL;
    GPIO_InitStruct.Alternate        = GPIO_AF0_MCO;
    HAL_GPIO_Init (IMU_INT_GPIO_Port, &GPIO_InitStruct);

    /* Enable EXTI interrupt [9:5] for IMU data ready interrupt */
    HAL_NVIC_SetPriority (IMU_INT_EXTI_IRQn, 8, 8);
    HAL_NVIC_EnableIRQ (IMU_INT_EXTI_IRQn);
#endif

    return eSTATUS_SUCCESS;
}

eSTATUS_t IMUStart (IMU* pIMU) {
    if (SENSOR_UPDATE_MODE_IS_INTERRUPT == TRUE) {
        if (IMUEnableInterrupts (pIMU) != eSTATUS_SUCCESS) {
            LOG_ERROR ("Failed to enable IMU interrupts");
            return eSTATUS_FAILURE;
        }
    }
    return eSTATUS_SUCCESS;
}

eSTATUS_t IMUProcessUpdatefromINT (IMU* pIMU, Vec3f* pOutputAccel, Vec3f* pOutputGyro) {
    if (pIMU == NULL) {
        LOG_ERROR ("IMU pointer is NULL");
        return (eSTATUS_t)eIMU_NULL_PTR;
    }
    if (pOutputAccel == NULL || pOutputGyro == NULL) {
        LOG_ERROR ("Output pointers are NULL");
        return (eSTATUS_t)eIMU_NULL_PTR;
    }

    if (pIMU->status != eSTATUS_SUCCESS) {
        if (IMUHandleErr (pIMU) != eSTATUS_SUCCESS) {
            LOG_ERROR ("Failed to handle IMU error");
            return eSTATUS_FAILURE;
        }
    }

    eSTATUS_t status = IMUConvertRaw (
    pIMU->aconf.range, pIMU->rawAccel, pIMU->gconf.range, pIMU->rawGyro,
    pOutputAccel, pOutputGyro);

    if (status != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to convert IMU raw data");
        return eSTATUS_FAILURE;
    }

    return eSTATUS_SUCCESS;
}

eSTATUS_t IMUProcessUpdatefromPolling (IMU* pIMU, Vec3f* pOutputAccel, Vec3f* pOutputGyro) {

    if (pIMU == NULL || pOutputAccel == NULL || pOutputGyro == NULL) {
        LOG_ERROR ("IMU or output pointers are NULL");
        return (eSTATUS_t)eIMU_NULL_PTR;
    }

    eSTATUS_t status = eSTATUS_SUCCESS;
    uint8_t accelRdy = FALSE;
    uint8_t gyroRdy  = FALSE;
    int32_t timeout  = 1000; // 1000ms
    while (timeout-- > 0) {
        uint16_t intStatus = 0; // BMI3_REG_STATUS
        status             = IMUGetStatusReg (pIMU, &intStatus);
        if (status != eSTATUS_SUCCESS) {
            LOG_ERROR ("Failed to read IMU interrupt status");
            return status;
        }

        if (BIT_ISSET (intStatus, STATUS_ACCEL_DATA_RDY_BIT) == TRUE && accelRdy == FALSE) {
            status = IMUUpdateAccel (pIMU);
            if (status != eSTATUS_SUCCESS) {
                LOG_ERROR ("Failed to update accelerometer data");
                return status;
            }
            accelRdy = TRUE;
        }

        if (BIT_ISSET (intStatus, STATUS_GYRO_DATA_RDY_BIT) == TRUE && gyroRdy == FALSE) {
            status = IMUUpdateGyro (pIMU);
            if (status != eSTATUS_SUCCESS) {
                LOG_ERROR ("Failed to update gyroscope data");
                return status;
            }
            gyroRdy = TRUE;
        }
        if (accelRdy && gyroRdy) {
            break;
        }
        HAL_Delay (1);
    }

    if (timeout > 0) {
        status = IMUConvertRaw (
        pIMU->aconf.range, pIMU->rawAccel, pIMU->gconf.range,
        pIMU->rawGyro, pOutputAccel, pOutputGyro);

        if (status != eSTATUS_SUCCESS) {
            LOG_ERROR ("Failed to convert raw IMU data");
            return status;
        }
    } else {
        LOG_ERROR ("IMU data not ready");
        return eSTATUS_FAILURE;
    }


    return eSTATUS_SUCCESS;
}

eSTATUS_t IMUStop (IMU* pIMU) {
    if (IMUDisableInterrupts (pIMU) != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to disable IMU interrupts");
        return eSTATUS_FAILURE;
    }
    return eSTATUS_SUCCESS;
}

/*
 * Attempts to handle IMU errors
 */
eSTATUS_t IMUHandleErr (IMU* pIMU) {

    IMUErr err = { 0 };
    if (IMUGetDeviceErr (pIMU, &err) != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to read IMU error codes");
        pIMU->status = eSTATUS_FAILURE;
        return eSTATUS_FAILURE;
    }

    IMULogDeviceErr (pIMU, &err);
    return eSTATUS_FAILURE;
}

void IMU2CPUInterruptHandler (IMU* pIMU) {

    eSTATUS_t status = eSTATUS_SUCCESS;
    if (pIMU == NULL || pIMU->pSPI == NULL) {
        status = (eSTATUS_t)eIMU_NULL_PTR;
        goto error;
    }

    // read both status registers
    uint8_t pBuf[2] = { 0 };
    status          = IMUReadReg (pIMU, BMI3_REG_INT_STATUS_INT1, pBuf, 2);
    if (status != eSTATUS_SUCCESS) {
        goto error;
    }

    uint16_t intStatus1 = ((uint16_t)pBuf[1]) << 8U | ((uint16_t)pBuf[0]);
    /* check if error status bit is set */
    if (BIT_ISSET (intStatus1, ((uint8_t)INT_ERR_STATUS_BIT << 1U))) {
        status = (eSTATUS_t)eIMU_HARDWARE_ERR;
        goto error;
    }

    /* check if accel data is ready */
    if (BIT_ISSET (intStatus1, INT1_ACCEL_DATA_RDY_BIT)) {
        status = IMUUpdateAccel (pIMU);
        if (status != eSTATUS_SUCCESS) {
            goto error;
        }
    }

    /* check if gyro data is ready */
    if (BIT_ISSET (intStatus1, INT1_GYRO_DATA_RDY_BIT)) {
        status = IMUUpdateGyro (pIMU);
        if (status != eSTATUS_SUCCESS) {
            goto error;
        }
    }

    /* check if temperature data is ready */
    if (BIT_ISSET (intStatus1, INT1_TEMP_DATA_RDY_BIT)) {
        // if (status != eSTATUS_SUCCESS) {
        //     goto error;
        // }
    };

error:
    pIMU->status = status;
}

eSTATUS_t IMUGetConf (IMU* pIMU, IMUAccConf* pAConf, IMUGyroConf* pGConf) {
    return IMUGetConf_ (pIMU, pAConf, pGConf, 0);
}

eSTATUS_t IMUGetAltConf (IMU* pIMU, IMUAccConf* pAConf, IMUGyroConf* pGConf) {
    return IMUGetConf_ (pIMU, pAConf, pGConf, 1);
}

eSTATUS_t IMUSetConf (IMU* pIMU, IMUAccConf const* pAConf, IMUGyroConf const* pGConf) {
    return IMUSetConf_ (pIMU, pAConf, pGConf, 0);
}

eSTATUS_t IMUSetAltConf (IMU* pIMU, IMUAccConf const* pAConf, IMUGyroConf const* pGConf) {
    return IMUSetConf_ (pIMU, pAConf, pGConf, TRUE);
}

eSTATUS_t IMUCompareConfs (IMUAccConf aconf, IMUGyroConf gconf, IMUAccConf aconf2, IMUGyroConf gconf2) {
    if (
    aconf.mode != aconf2.mode || aconf.odr != aconf2.odr ||
    aconf.range != aconf2.range || aconf.avg != aconf2.avg ||
    aconf.bw != aconf2.bw) {
        return eSTATUS_FAILURE;
    }
    if (
    gconf.mode != gconf2.mode || gconf.odr != gconf2.odr ||
    gconf.range != gconf2.range || gconf.avg != gconf2.avg ||
    gconf.bw != gconf2.bw) {
        return eSTATUS_FAILURE;
    }
    return eSTATUS_SUCCESS;
}
