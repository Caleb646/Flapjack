#include "sensors/imu/imu.h"
#include "common.h"
#include "log.h"
#include "sensors/imu/bmixxx.h"
// #include "stm32h7xx_hal.h"
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

STATUS_TYPE IMUSendCmd (IMU const* pIMU, uint16_t cmd) {
    uint8_t pRegData[2] = { 0 };
    pRegData[0]         = (uint8_t)(cmd & BMI3_SET_LOW_BYTE);
    pRegData[1]         = (uint8_t)((cmd & BMI3_SET_HIGH_BYTE) >> 8U);
    STATUS_TYPE status  = IMUWriteReg (pIMU, BMI3_REG_CMD, pRegData, 2);
    if (status != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to send IMU command 0x%04X", cmd);
        return status;
    }
    return eSTATUS_SUCCESS;
}

STATUS_TYPE
IMUGetFeatureStatus (IMU const* pIMU, uint16_t featureRegAddr, IMUFeatureStatus* pResultOut) {
    uint8_t pData[2]   = { 0 };
    STATUS_TYPE status = IMUReadReg (pIMU, featureRegAddr, pData, 2);
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

STATUS_TYPE IMUGetINTStatus (IMU const* pIMU, uint16_t* pOutStatus) {
    uint8_t pBuff[2] = { 0U };
    STATUS_TYPE status = IMUReadReg (pIMU, BMI3_REG_INT_STATUS_INT1, pBuff, 2);
    if (status != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to read IMU interrupt status register");
        return status;
    }
    *pOutStatus = ((uint16_t)pBuff[1] << 8U) | (uint16_t)pBuff[0];
    return eSTATUS_SUCCESS;
}

STATUS_TYPE IMUGetStatusReg (IMU const* pIMU, uint16_t* pOutStatus) {
    uint8_t pBuff[2]   = { 0U };
    STATUS_TYPE status = IMUReadReg (pIMU, BMI3_REG_STATUS, pBuff, 2);
    if (status != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to read IMU status register");
        return status;
    }
    *pOutStatus = ((uint16_t)pBuff[1] << 8U) | (uint16_t)pBuff[0];
    return eSTATUS_SUCCESS;
}

STATUS_TYPE IMUGetDeviceErr (IMU* pIMU, IMUErr* pOutErr) {
    uint8_t pBuff[2]   = { 0U };
    STATUS_TYPE status = IMUReadReg (pIMU, BMI3_REG_ERR_REG, pBuff, 2);
    uint16_t err       = ((uint16_t)pBuff[1] << 8U) | (uint16_t)pBuff[0];
    pOutErr->err       = err;
    pOutErr->fatalErr  = (err & (1U << 0U)) > 0;
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
    // /* Clear IMU error status on successful read */
    // pIMU->status = eSTATUS_SUCCESS;
    return eSTATUS_SUCCESS; //
}

void IMULogDeviceErr (IMU* pIMU, IMUErr const* pErr) {
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

/*
 * Attempts to handle IMU errors via soft reset
 */
STATUS_TYPE IMUHandleErr (IMU* pIMU) {

    IMUErr err = { 0 };
    if (IMUGetDeviceErr (pIMU, &err) != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to read IMU error codes");
        pIMU->status = eSTATUS_FAILURE;
        return eSTATUS_FAILURE;
    }

    IMULogDeviceErr (pIMU, &err);
    return eSTATUS_FAILURE;
}

STATUS_TYPE IMUReadReg (IMU const* pIMU, uint8_t reg, uint8_t* pBuf, uint32_t len) {

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
        return (STATUS_TYPE)eIMU_RW_BUFFER_OVERFLOW;
    }

    if (HAL_SPI_TransmitReceive (pIMU->pSPI, pTx, pRx, len + pIMU->nDummyBytes + 1, SPI_DEFAULT_TIMEOUT_MS) != HAL_OK) {
        return (STATUS_TYPE)eIMU_COM_FAILURE;
    }

    // Add 2 microsecond delay after IMU read operation
    DelayMicroseconds (2);

    // The first nDummyBytes + 1 (for the register address) are dummy bytes
    memcpy (pBuf, &(pRx[pIMU->nDummyBytes + 1]), len);
    return eSTATUS_SUCCESS;
}

STATUS_TYPE IMUWriteReg (IMU const* pIMU, uint8_t reg, uint8_t* pBuf, uint32_t len) {

    uint8_t pTx[RW_BUFFER_SZ] = { 0 };
    if (len + 1 > RW_BUFFER_SZ) {
        return (STATUS_TYPE)eIMU_RW_BUFFER_OVERFLOW;
    }

    pTx[0] = reg & BMI3_SPI_WR_MASK;
    memcpy (&pTx[1], (void*)pBuf, len);

    if (HAL_SPI_Transmit (pIMU->pSPI, pTx, len + 1, SPI_DEFAULT_TIMEOUT_MS) != HAL_OK) {
        return (STATUS_TYPE)eIMU_COM_FAILURE;
    }

    // Add 2 microsecond delay after IMU write operation
    DelayMicroseconds (2);

    return eSTATUS_SUCCESS;
}

STATUS_TYPE IMUUpdateGyro (IMU* pIMU) {

    uint8_t pBuffer[6] = { 0 };
    STATUS_TYPE status = IMUReadReg (pIMU, BMI3_REG_GYR_DATA_X, pBuffer, 6);
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

STATUS_TYPE IMUUpdateAccel (IMU* pIMU) { // , Vec3 curVel, Vec3* pOutputAccel) {
    uint8_t pBuffer[6] = { 0 };
    STATUS_TYPE status = IMUReadReg (pIMU, BMI3_REG_ACC_DATA_X, pBuffer, 6);
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

STATUS_TYPE IMU2CPUInterruptHandler (IMU* pIMU) {
    if (pIMU == NULL || pIMU->pSPI == NULL) {
        return (STATUS_TYPE)eIMU_NULL_PTR;
    }

    // read both status registers
    uint8_t pBuf[2] = { 0 };
    STATUS_TYPE status = IMUReadReg (pIMU, BMI3_REG_INT_STATUS_INT1, pBuf, 2);

    if (status != eSTATUS_SUCCESS) {
        return status;
    }

    uint16_t intStatus1 = ((uint16_t)pBuf[1]) << 8U | ((uint16_t)pBuf[0]);
    /* check if error status bit is set */
    if (BIT_ISSET (intStatus1, ((uint8_t)INT_ERR_STATUS_BIT << 1U))) {
        return (STATUS_TYPE)eIMU_HARDWARE_ERR;
    }
    // LOG_INFO ("IMU interrupt status: 0x%04X", intStatus1);
    /* check if accel data is ready */
    if (BIT_ISSET (intStatus1, INT1_ACCEL_DATA_RDY_BIT)) {
        // LOG_INFO ("IMU accel data ready");
        status = IMUUpdateAccel (pIMU);
        if (status != eSTATUS_SUCCESS) {
            return status;
        }
    }

    /* check if gyro data is ready */
    if (BIT_ISSET (intStatus1, INT1_GYRO_DATA_RDY_BIT)) {
        status = IMUUpdateGyro (pIMU);
        if (status != eSTATUS_SUCCESS) {
            return status;
        }
    }
    /* check if temperature data is ready */
    if (BIT_ISSET (intStatus1, INT1_TEMP_DATA_RDY_BIT)) {
        // if (status != eSTATUS_SUCCESS) {
        //     return status;
        // }
    };
    return status;
}

STATUS_TYPE IMUPollData (IMU* pIMU, Vec3f* pOutputAccel, Vec3f* pOutputGyro) {

    if (pIMU == NULL || pOutputAccel == NULL || pOutputGyro == NULL) {
        LOG_ERROR ("IMU or output pointers are NULL");
        return (STATUS_TYPE)eIMU_NULL_PTR;
    }

    STATUS_TYPE status = eSTATUS_SUCCESS;
    uint8_t accelRdy   = FALSE;
    uint8_t gyroRdy    = FALSE;
    int32_t timeout    = 1000; // 1000ms
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

STATUS_TYPE IMUSoftReset (IMU* pIMU) {
    /* Send soft reset command to BMI323 */
    STATUS_TYPE status = IMUSendCmd (pIMU, BMI3_CMD_SOFT_RESET);

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

static STATUS_TYPE
IMUGetConf_ (IMU* pIMU, IMUAccConf* pAConf, IMUGyroConf* pGConf, uint8_t altConfFlag) {
    STATUS_TYPE status = eSTATUS_SUCCESS;
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

STATUS_TYPE IMUGetConf (IMU* pIMU, IMUAccConf* pAConf, IMUGyroConf* pGConf) {
    return IMUGetConf_ (pIMU, pAConf, pGConf, 0);
}

STATUS_TYPE IMUGetAltConf (IMU* pIMU, IMUAccConf* pAConf, IMUGyroConf* pGConf) {
    return IMUGetConf_ (pIMU, pAConf, pGConf, 1);
}

static STATUS_TYPE
IMUSetConf_ (IMU* pIMU, IMUAccConf const* pAConf, IMUGyroConf const* pGConf, uint8_t altConfFlag) {
    /* Configure Accelerometer */
    STATUS_TYPE status = eSTATUS_SUCCESS;
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

STATUS_TYPE
IMUSetConf (IMU* pIMU, IMUAccConf const* pAConf, IMUGyroConf const* pGConf) {
    return IMUSetConf_ (pIMU, pAConf, pGConf, 0);
}

STATUS_TYPE
IMUSetAltConf (IMU* pIMU, IMUAccConf const* pAConf, IMUGyroConf const* pGConf) {
    return IMUSetConf_ (pIMU, pAConf, pGConf, TRUE);
}

STATUS_TYPE
IMUCompareConfs (IMUAccConf aconf, IMUGyroConf gconf, IMUAccConf aconf2, IMUGyroConf gconf2) {
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


STATUS_TYPE
IMUCalibrate (IMU* pIMU, uint8_t calibSelection, uint8_t applyCorrection, IMUSelfCalibResult* pResultOut) {

    if (pIMU == NULL || pResultOut == NULL) {
        LOG_ERROR ("IMU or result pointer is NULL");
        return (STATUS_TYPE)eIMU_NULL_PTR;
    }

    pResultOut->result = FALSE;

    /* Save the current configs */
    IMUAccConf aconf;
    IMUGyroConf gconf;
    STATUS_TYPE status = IMUGetConf (pIMU, &aconf, &gconf);
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

STATUS_TYPE IMUSetupInterrupts (IMU const* pIMU) {
    uint8_t pRegData[4] = { 0 };
    uint16_t temp       = 0;
    /* Map all enabled interrupts to pin INT1 */
    uint8_t enable     = BMI3_INT1;
    uint8_t disable    = BMI3_INT_NONE;
    STATUS_TYPE status = IMUReadReg (pIMU, BMI3_REG_INT_MAP1, pRegData, 4);

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

STATUS_TYPE IMUEnableInterrupts (IMU const* pIMU) {
    if (pIMU == NULL || pIMU->pSPI == NULL) {
        return (STATUS_TYPE)eIMU_NULL_PTR;
    }

    // Enable INT1 and INT2 with active high
    uint8_t pEnableInterrupts[2] = { (1U << 2U | 1U << 0U), (1U << 2U | 1U << 0U) };
    STATUS_TYPE status =
    IMUWriteReg (pIMU, BMI3_REG_IO_INT_CTRL, pEnableInterrupts, 2);

    return status;
}

STATUS_TYPE IMUDisableInterrupts (IMU const* pIMU) {
    if (pIMU == NULL || pIMU->pSPI == NULL) {
        return (STATUS_TYPE)eIMU_NULL_PTR;
    }

    // Enable INT1 and INT2 with active high
    uint8_t pDisableInterrupts[2] = { 0, 0 };
    STATUS_TYPE status =
    IMUWriteReg (pIMU, BMI3_REG_IO_INT_CTRL, pDisableInterrupts, 2);

    return status;
}

STATUS_TYPE
IMUConvertRaw (IMU_ACC_RANGE aRange, Vec3 ra, IMU_GYRO_RANGE gRange, Vec3 rg, Vec3f* pAccelOut, Vec3f* pGyroOut) {

    if (pAccelOut == NULL || pGyroOut == NULL) {
        return (STATUS_TYPE)eIMU_NULL_PTR;
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


STATUS_TYPE IMUInit (IMU* pIMU, SPI_HandleTypeDef* pSPI, IMUAccConf aconf, IMUGyroConf gconf) {

    memset (pIMU, 0, sizeof (IMU));
    pIMU->pSPI                 = pSPI;
    pIMU->msLastAccUpdateTime  = HAL_GetTick ();
    pIMU->msLastGyroUpdateTime = HAL_GetTick ();
    /* SPI reads have 1 dummy byte at the beginning */
    pIMU->nDummyBytes = 1;
    pIMU->magic       = IMU_MAGIC;

    /*
     * Soft reset IMU and switch to SPI
     */
    STATUS_TYPE status = IMUSoftReset (pIMU);
    if (status != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to soft reset IMU");
        return status;
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
        LOG_ERROR (
        "Failed to find BMI323. Chip ID [0x%X] [0x%X] is incorrect",
        pChipID[0], pChipID[1]);
        // LOG_ARRAY (pChipID, 2, "0x%X");
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

    return eSTATUS_SUCCESS;
}

STATUS_TYPE IMUStart (IMU* pIMU) {
    if (IMUEnableInterrupts (pIMU) != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to enable IMU interrupts");
        return eSTATUS_FAILURE;
    }
    return eSTATUS_SUCCESS;
}
STATUS_TYPE IMUStop (IMU* pIMU) {
    if (IMUDisableInterrupts (pIMU) != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to disable IMU interrupts");
        return eSTATUS_FAILURE;
    }
    return eSTATUS_SUCCESS;
}
