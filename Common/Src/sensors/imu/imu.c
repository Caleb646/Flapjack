#include "sensors/imu/imu.h"
#include "common.h"
#include "log.h"
#include "sensors/imu/bmixxx.h"
// #include "stm32h7xx_hal.h"
#include <stdint.h>
#include <string.h>


#define BIT_ISSET(v, bit) (((v) & (bit)) > 0U)

enum {
    RW_BUFFER_SZ           = 16U,
    INT_ERR_STATUS_BIT     = 10U,
    SPI_DEFAULT_TIMEOUT_MS = 100U
};

STATUS_TYPE IMUGetErr (IMU* pIMU, IMUErr* pOutErr) {
    uint8_t pBuff[2]   = { 0U };
    STATUS_TYPE status = IMUReadReg (pIMU, BMI3_REG_ERR_REG, pBuff, 2);
    uint16_t err       = ((uint16_t)pBuff[1] << 8U) | (uint16_t)pBuff[0];
    pOutErr->fatalErr  = (err & (1U << 0U)) > 0;
    pOutErr->featEngOvrld = (err & (1U << 2U)) > 0;
    pOutErr->featEngWd    = (err & (1U << 4U)) > 0;
    pOutErr->accConfErr   = (err & (1U << 5U)) > 0;
    pOutErr->gyrConfErr   = (err & (1U << 6U)) > 0;
    pOutErr->i3cErr0      = (err & (1U << 8U)) > 0;
    pOutErr->i3cErr1      = (err & (1U << 11U)) > 0;
    if (status != eSTATUS_SUCCESS) {
        return status;
    }
    /* Clear IMU error status on successful read */
    pIMU->status = eSTATUS_SUCCESS;
    return eSTATUS_SUCCESS; //
}

void IMULogErr (STATUS_TYPE curImuStatus, IMUErr const* pOutErr) {
    if (curImuStatus == (STATUS_TYPE)eIMU_COM_FAILURE) {
        LOG_ERROR (
        "IMU Communication failure error. It occurs due to "
        "read/write operation failure and also due to power failure "
        "during communication");
    }
    if (curImuStatus == (STATUS_TYPE)eIMU_RW_BUFFER_OVERFLOW) {
        LOG_ERROR ("IMU register read or write buffer size was exceeded");
    }
    if (curImuStatus == (STATUS_TYPE)eIMU_NULL_PTR) {
        LOG_ERROR ("IMU received null ptr");
    }
    if (pOutErr->fatalErr != 0) {
        LOG_ERROR (
        "IMU fatal error, chip is not in operation state (Boot or "
        "Power-System). Power on reset or soft reset required");
    }
    if (pOutErr->featEngOvrld != 0) {
        LOG_ERROR ("IMU overload of the feature engine detected");
    }
    if (pOutErr->featEngWd != 0) {
        LOG_ERROR ("IMU watchdog timer of the feature engine triggered");
    }
    if (pOutErr->accConfErr != 0) {
        LOG_ERROR ("IMU unsupported accelerometer configuration set by user");
    }
    if (pOutErr->gyrConfErr != 0) {
        LOG_ERROR ("IMU unsupported gyroscope configuration set by user");
    }
    if (pOutErr->i3cErr0 != 0) {
        LOG_ERROR ("IMU I3C SDR parity error occurred");
    }
    if (pOutErr->i3cErr1 != 0) {
        LOG_ERROR ("IMU I3C S0/S1 error occurred");
    }
}

STATUS_TYPE IMUReadReg (IMU const* pIMU, uint8_t reg, uint8_t* pBuf, uint32_t len) {
    uint8_t pTx[RW_BUFFER_SZ] = { 0 };
    // set read mask for register address
    pTx[0] = BMI3_SPI_RD_MASK | reg;

    uint8_t pRx[RW_BUFFER_SZ] = { 0 };
    // set NSS low
    // HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_RESET);
    // status = HAL_SPI_Transmit(pIMUSPIRef, pTemp, 1, 100);

    // set NSS high
    // HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_SET);

    if (len + pIMU->nDummyBytes > RW_BUFFER_SZ) {
        return (STATUS_TYPE)eIMU_RW_BUFFER_OVERFLOW;
    }

    if (HAL_SPI_TransmitReceive (pIMU->pSPI, pTx, pRx, len + pIMU->nDummyBytes, SPI_DEFAULT_TIMEOUT_MS) != HAL_OK) {
        return (STATUS_TYPE)eIMU_COM_FAILURE;
    }
    // The first nDummyBytes are dummy bytes
    memcpy (pBuf, &pRx[pIMU->nDummyBytes], len);
    return eSTATUS_SUCCESS;
}

STATUS_TYPE IMUWriteReg (IMU const* pIMU, uint8_t reg, uint8_t* pBuf, uint32_t len) {
    uint8_t pTx[RW_BUFFER_SZ] = { 0 };

    if (len + 1 > RW_BUFFER_SZ) {
        return (STATUS_TYPE)eIMU_RW_BUFFER_OVERFLOW;
    }

    pTx[0] = reg | BMI3_SPI_WR_MASK;
    memcpy (&pTx[1], (void*)pBuf, len);

    if (HAL_SPI_Transmit (pIMU->pSPI, pTx, len + 1, SPI_DEFAULT_TIMEOUT_MS) != HAL_OK) {
        return (STATUS_TYPE)eIMU_COM_FAILURE;
    }
    return eSTATUS_SUCCESS;
}

STATUS_TYPE IMUUpdateGyro (IMU* pIMU, Vec3 curAngularVel, Vec3* pOutputGyro) {
    uint8_t pBuffer[6] = { 0 };
    STATUS_TYPE status = IMUReadReg (pIMU, BMI3_REG_GYR_DATA_X, pBuffer, 6);
    if (status != eSTATUS_SUCCESS) {
        return status;
    }

    pIMU->rawGyro.x =
    (int32_t)(((uint16_t)pBuffer[1]) << 8U) | ((uint16_t)pBuffer[0]);
    pIMU->rawGyro.y =
    (int32_t)(((uint16_t)pBuffer[3]) << 8U) | ((uint16_t)pBuffer[2]);
    pIMU->rawGyro.z =
    (int32_t)(((uint16_t)pBuffer[5]) << 8U) | ((uint16_t)pBuffer[4]);

    int32_t scale = 125;
    if (pIMU->gconf.range == eIMU_GYRO_RANGE_250) {
        scale = 250;
    }
    if (pIMU->gconf.range == eIMU_GYRO_RANGE_500) {
        scale = 500;
    }
    if (pIMU->gconf.range == eIMU_GYRO_RANGE_1000) {
        scale = 1000;
    }
    if (pIMU->gconf.range == eIMU_GYRO_RANGE_2000) {
        scale = 2000;
    }

    // scale to milli degrees per second
    pOutputGyro->x = (((int32_t)pIMU->rawGyro.x) * 1000 * scale) / 0x7FFF;
    pOutputGyro->y = (((int32_t)pIMU->rawGyro.y) * 1000 * scale) / 0x7FFF;
    pOutputGyro->z = (((int32_t)pIMU->rawGyro.z) * 1000 * scale) / 0x7FFF;

    // pOutputAngularVel->x = ( ((int32_t)pIMU->rawGyro.x) * 1000 * scale ) / 0x7FFF;
    // pOutputAngularVel->y = ( ((int32_t)pIMU->rawGyro.y) * 1000 * scale ) / 0x7FFF;
    // pOutputAngularVel->z = ( ((int32_t)pIMU->rawGyro.z) * 1000 * scale ) / 0x7FFF;

    // pOutputGyro->x = pOutputAngularVel->x;
    // pOutputGyro->y = pOutputAngularVel->y;
    // pOutputGyro->z = pOutputAngularVel->z;

    pIMU->msLastGyroUpdateTime = HAL_GetTick ();

    return eSTATUS_SUCCESS;
}

STATUS_TYPE IMUUpdateAccel (IMU* pIMU, Vec3 curVel, Vec3* pOutputAccel) {
    uint8_t pBuffer[6] = { 0 };
    STATUS_TYPE status = IMUReadReg (pIMU, BMI3_REG_ACC_DATA_X, pBuffer, 6);
    if (status != eSTATUS_SUCCESS) {
        return status;
    }

    pIMU->rawAccel.x =
    (int32_t)(((uint16_t)pBuffer[1]) << 8U) | ((uint16_t)pBuffer[0]);
    pIMU->rawAccel.y =
    (int32_t)(((uint16_t)pBuffer[3]) << 8U) | ((uint16_t)pBuffer[2]);
    pIMU->rawAccel.z =
    (int32_t)(((uint16_t)pBuffer[5]) << 8U) | ((uint16_t)pBuffer[4]);

    int32_t scale = 2;
    if (pIMU->aconf.range == eIMU_ACC_RANGE_4G) {
        scale = 4;
    }
    if (pIMU->aconf.range == eIMU_ACC_RANGE_8G) {
        scale = 8;
    }
    if (pIMU->aconf.range == eIMU_ACC_RANGE_16G) {
        scale = 16;
    }
    // scale to millimeters per second ^ 2
    int32_t ax = (((int32_t)pIMU->rawAccel.x) * 1000 * scale) / 0x7FFF;
    int32_t ay = (((int32_t)pIMU->rawAccel.y) * 1000 * scale) / 0x7FFF;
    int32_t az = (((int32_t)pIMU->rawAccel.z) * 1000 * scale) / 0x7FFF;

    // int32_t ms = HAL_GetTick();
    // int32_t dt = ms - pIMU->msLastAccUpdateTime;
    // pIMU->msLastAccUpdateTime = ms;

    pOutputAccel->x = ax;
    pOutputAccel->y = ay;
    pOutputAccel->z = az;

    pIMU->msLastAccUpdateTime = HAL_GetTick ();

    // pOutputVel->x = curVel.x + ((ax * dt) / 1000);
    // pOutputVel->y = curVel.y + ((ay * dt) / 1000);
    // pOutputVel->z = curVel.z + ((az * dt) / 1000);

    return eSTATUS_SUCCESS;
}

STATUS_TYPE IMU2CPUInterruptHandler (IMU* pIMU, Vec3* pOutputAccel, Vec3* pOutputGyro) {
    if (pIMU == NULL || pIMU->pSPI == NULL || pOutputAccel == NULL || pOutputGyro == NULL) {
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

    /* check if accel data is ready */
    if (BIT_ISSET (intStatus1, (13U << 1U))) {
        status = IMUUpdateAccel (pIMU, *pOutputAccel, pOutputAccel);
        if (status != eSTATUS_SUCCESS) {
            return status;
        }
    }
    /* check if gyro data is ready */
    if (BIT_ISSET (intStatus1, (12U << 1U))) {
        status = IMUUpdateGyro (pIMU, *pOutputGyro, pOutputGyro);
        if (status != eSTATUS_SUCCESS) {
            return status;
        }
    }
    /* check if temperature data is ready */
    if (BIT_ISSET (intStatus1, (11U << 1U))) {
        // if (status != eSTATUS_SUCCESS) {
        //     return status;
        // }
    };
    return status;
}

STATUS_TYPE IMUSoftReset (IMU* pIMU) {
    /* Send soft reset command to BMI323 */
    uint8_t cmdBuffer[2] = { 0 };
    cmdBuffer[0] = (uint8_t)(BMI3_CMD_SOFT_RESET & BMI3_SET_LOW_BYTE);
    cmdBuffer[1] = (uint8_t)((BMI3_CMD_SOFT_RESET & BMI3_SET_HIGH_BYTE) >> 8);
    STATUS_TYPE status = IMUWriteReg (pIMU, BMI3_REG_CMD, cmdBuffer, 2);

    /* Perform dummy read to switch from I3C/I2C to SPI */
    if (status == eSTATUS_SUCCESS) {
        uint8_t dummyBytes[2] = { 0 };
        status = IMUReadReg (pIMU, BMI3_REG_CHIP_ID, dummyBytes, 2);
    }

    /* Enable feature engine */
    if (status == eSTATUS_SUCCESS) {
        uint8_t featureData[2] = { 0x2c, 0x01 };
        status = IMUWriteReg (pIMU, BMI3_REG_FEATURE_IO2, featureData, 2);
    }

    /* Enable feature status bit */
    if (status == eSTATUS_SUCCESS) {
        uint8_t featureIOStatus[2] = { BMI3_ENABLE, 0 };
        status = IMUWriteReg (pIMU, BMI3_REG_FEATURE_IO_STATUS, featureIOStatus, 2);
    }

    /* Enable feature engine bit */
    if (status == eSTATUS_SUCCESS) {
        uint8_t featureEngine[2] = { BMI3_ENABLE, 0 };
        status = IMUWriteReg (pIMU, BMI3_REG_FEATURE_CTRL, featureEngine, 2);
    }

    if (status == eSTATUS_SUCCESS) {
        int16_t loop       = 0;
        uint8_t regData[2] = { 0 };

        while (loop++ <= 10) {
            HAL_Delay (100);
            status = IMUReadReg (pIMU, BMI3_REG_FEATURE_IO1, regData, 2);
            if (status == eSTATUS_SUCCESS) {
                if (regData[0] & (uint16_t)BMI3_FEATURE_ENGINE_ENABLE_MASK) {
                    status = eSTATUS_SUCCESS;
                    break;
                }
            }
        }
    }

    return status;
}

static STATUS_TYPE
IMUGetConf_ (IMU* pIMU, IMUAccConf* pAConf, IMUGyroConf* pGConf, uint8_t altConfFlag) {
    STATUS_TYPE status = eSTATUS_SUCCESS;
    /* Accelerometer Config */
    if (pAConf != NULL) {
        uint8_t regAddr = BMI3_REG_ACC_CONF;
        if (altConfFlag) {
            regAddr = BMI3_REG_ALT_ACC_CONF;
        }
        uint8_t data[2] = { 0 };
        status          = IMUReadReg (pIMU, regAddr, data, 2);
        if (status != eSTATUS_SUCCESS) {
            return status;
        }
        uint8_t conf = data[0];
        // NOLINTBEGIN(hicpp-signed-bitwise)
        pAConf->odr   = BMI3_GET_BIT_POS0 (conf, BMI3_ACC_ODR);
        pAConf->range = BMI3_GET_BITS (conf, BMI3_ACC_RANGE);
        pAConf->bw    = BMI3_GET_BITS (conf, BMI3_ACC_BW);

        conf         = ((uint16_t)data[1]) << 8U;
        pAConf->avg  = BMI3_GET_BITS (conf, BMI3_ACC_AVG_NUM);
        pAConf->mode = BMI3_GET_BITS (conf, BMI3_ACC_MODE);

        if (altConfFlag) {
            uint8_t conf = data[0];
            pAConf->odr  = BMI3_GET_BIT_POS0 (conf, BMI3_ALT_ACC_ODR);

            conf         = ((uint16_t)data[1]) << 8U;
            pAConf->avg  = BMI3_GET_BITS (conf, BMI3_ALT_ACC_AVG_NUM);
            pAConf->mode = BMI3_GET_BITS (conf, BMI3_ALT_ACC_MODE);
        }
        // NOLINTEND(hicpp-signed-bitwise)
    }
    /* Gyro Config */
    if (pGConf != NULL) {
        uint8_t regAddr = BMI3_REG_GYR_CONF;
        if (altConfFlag) {
            regAddr = BMI3_REG_ALT_GYR_CONF;
        }
        uint8_t data[2] = { 0 };
        status          = IMUReadReg (pIMU, regAddr, data, 2);
        if (status != eSTATUS_SUCCESS) {
            return status;
        }
        uint8_t conf = data[0];
        // NOLINTBEGIN(hicpp-signed-bitwise)
        pGConf->odr   = BMI3_GET_BIT_POS0 (conf, BMI3_GYR_ODR);
        pGConf->range = BMI3_GET_BITS (conf, BMI3_GYR_RANGE);
        pGConf->bw    = BMI3_GET_BITS (conf, BMI3_GYR_BW);

        conf         = ((uint16_t)data[1]) << 8U;
        pGConf->avg  = BMI3_GET_BITS (conf, BMI3_GYR_AVG_NUM);
        pGConf->mode = BMI3_GET_BITS (conf, BMI3_GYR_MODE);

        if (altConfFlag) {
            uint8_t conf = data[0];
            pGConf->odr  = BMI3_GET_BIT_POS0 (conf, BMI3_ALT_GYR_ODR);

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

        if (altConfFlag) {
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
        if (status == eSTATUS_SUCCESS && !altConfFlag) {
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

        if (altConfFlag) {
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
        if (status == eSTATUS_SUCCESS && !altConfFlag) {
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
    return IMUSetConf_ (pIMU, pAConf, pGConf, 1);
}


STATUS_TYPE
IMUCalibrate (IMU* pIMU, uint8_t calibSelection, uint8_t applyCorrection, IMUSelfCalibResult* pResultOut) {

    /* Save the current configs */
    IMUAccConf aconf;
    IMUGyroConf gconf;
    STATUS_TYPE status = IMUGetConf (pIMU, &aconf, &gconf);
    if (status != eSTATUS_SUCCESS) {
        return status;
    }

    /* Set the ACC config to be what the self calibration expects */
    IMUAccConf calibAConf = { 0 };
    calibAConf.mode       = eIMU_ACC_MODE_HIGH_PERF;
    calibAConf.odr        = eIMU_ACC_ODR_100;
    calibAConf.range      = eIMU_ACC_RANGE_8G;
    calibAConf.avg        = eIMU_ACC_AVG_1;
    calibAConf.bw         = eIMU_ACC_BW_HALF;
    status                = IMUSetConf (pIMU, &calibAConf, NULL);
    if (status != eSTATUS_SUCCESS) {
        return status;
    }

    /* Store alt configs and then disable them */
    IMUAccConf altAConf;
    IMUGyroConf altGConf;
    status = IMUGetAltConf (pIMU, &altAConf, &altGConf);
    if (status != eSTATUS_SUCCESS) {
        return status;
    }

    altAConf.mode = eIMU_ACC_MODE_DISABLE;
    altGConf.mode = eIMU_GYRO_MODE_DISABLE;
    status        = IMUSetAltConf (pIMU, &altAConf, &altGConf);
    if (status != eSTATUS_SUCCESS) {
        return status;
    }

    /* Set the calibration mode in the dma register */
    {
        uint8_t pRegData[2]      = { 0 };
        uint8_t calibBaseAddr[2] = { BMI3_BASE_ADDR_GYRO_SC_SELECT, 0 };
        status = IMUWriteReg (pIMU, BMI3_REG_FEATURE_DATA_ADDR, calibBaseAddr, 2);
        if (status != eSTATUS_SUCCESS) {
            return status;
        }
        status = IMUReadReg (pIMU, BMI3_REG_FEATURE_DATA_TX, pRegData, 2);
        pRegData[0] = (applyCorrection | calibSelection);
        if (status != eSTATUS_SUCCESS) {
            return status;
        }
        status = IMUWriteReg (pIMU, BMI3_REG_FEATURE_DATA_ADDR, calibBaseAddr, 2);
        if (status != eSTATUS_SUCCESS) {
            return status;
        }
        status = IMUWriteReg (pIMU, BMI3_REG_FEATURE_DATA_TX, pRegData, 2);
        if (status != eSTATUS_SUCCESS) {
            return status;
        }
    }

    /* Trigger the self calibration */
    {
        uint8_t pRegData[2] = { 0 };
        pRegData[0] = (uint8_t)(BMI3_CMD_SELF_CALIB_TRIGGER & BMI3_SET_LOW_BYTE);
        pRegData[1] =
        (uint8_t)((BMI3_CMD_SELF_CALIB_TRIGGER & BMI3_SET_HIGH_BYTE) >> 8U);
        status = IMUWriteReg (pIMU, BMI3_REG_CMD, pRegData, 2);
        if (status != eSTATUS_SUCCESS) {
            return status;
        }
    }
    /* Get self calibration result */
    {
        STATUS_TYPE status = eSTATUS_SUCCESS;
        uint8_t idx        = 0;
        uint8_t limit      = 10;
        uint8_t sc_status  = 0;
        uint8_t pData[2];
        uint8_t sc_complete_flag = 0;
        // uint8_t feature_engine_err_reg_lsb, feature_engine_err_reg_msb;
        pResultOut->error = 0;

        for (idx = 0; idx < limit; idx++) {
            /* A delay of 430ms (43ms * 10(limit)) is required to perform self calibration */
            HAL_Delay (43);
            status = IMUReadReg (pIMU, BMI3_REG_FEATURE_IO1, pData, 2);
            sc_status = (pData[0] & BMI3_SC_ST_STATUS_MASK) >> BMI3_SC_ST_COMPLETE_POS;

            if ((sc_status == BMI3_TRUE) && (status == eSTATUS_SUCCESS)) {
                /*Bail early if the self-calibration is completed * / */
                sc_complete_flag = BMI3_TRUE;
                break;
            }
        }

        if (sc_complete_flag == BMI3_TRUE) {
            pResultOut->result =
            (pData[0] & BMI3_GYRO_SC_RESULT_MASK) >> BMI3_GYRO_SC_RESULT_POS;
            pResultOut->error = pData[0];
        } else {
            /* If limit elapses returning the error code, error status is returned */
            // rslt = bmi3_get_feature_engine_error_status (
            // &feature_engine_err_reg_lsb, &feature_engine_err_reg_msb,
            // dev); sc_rslt->sc_error_status = feature_engine_err_reg_lsb;
        }
    }
    /* Restore configs */
    status = IMUSetConf (pIMU, &aconf, &gconf);
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
        LOG_ERROR ("IMU failed to configure accel or gyro ");
        return status;
    }

    uint8_t pChipID[2] = { 0 };
    status             = IMUReadReg (pIMU, BMI3_REG_CHIP_ID, pChipID, 2);
    if (pChipID[0] != BMI323_CHIP_ID) {
        LOG_ERROR ("Failed to find BMI323. Chip ID [%X] is incorrect", pChipID[0]);
        return eSTATUS_FAILURE;
    }

    /* Enable acc, gyro, and temperature - data ready interrupts for pin INT1 */
    status = IMUSetupInterrupts (pIMU);
    if (status != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to setup IMU interrupts");
        return eSTATUS_FAILURE;
    }

    // // Disable PWR_CONF advanced power save
    // pBuffer[0] = 0;
    // status = IMUWriteReg(pIMU, BMI2_PWR_CONF_ADDR, pBuffer, 1);

    // HAL_Delay(1);

    // // Prepare config file
    // pBuffer[0] = 0;
    // status = IMUWriteReg(pIMU, BMI2_INIT_CTRL_ADDR, pBuffer, 1);

    // // Added the data write address directly to the config_file
    // HAL_SPI_Transmit(pIMU->pSPI, bmi270_config_file, sizeof(bmi270_config_file), 100);

    // pBuffer[0] = 0x01;
    // status = IMUWriteReg(pIMU, BMI2_INIT_CTRL_ADDR, pBuffer, 1);

    // HAL_Delay(20);

    // status = IMUReadReg(pIMU, BMI2_INTERNAL_STATUS_ADDR, pBuffer, 1);
    // if(status == eSTATUS_FAILURE || (pBuffer[0] & 1) == 0)
    // {
    //     LOG_ERROR(
    //     "IMU failed to read status or returned status [%X] is invalid", pBuffer[0]
    //     );
    // 	return eSTATUS_FAILURE;
    // }

    // /*
    // * Device Enable/Disable
    // */
    // // enable acceleration, gyro, and temp but disable auxillary interface
    // pBuffer[0] = 0x0E;
    // status = IMUWriteReg(pIMU, BMI2_PWR_CTRL_ADDR, pBuffer, 1);

    // /*
    // * Accelerometer Setup
    // */
    // // enable acc filter perf bit, set acc bwp to normal, and set acc_odr to 100 Hz
    // pBuffer[0] = BMI2_ACC_CONF_PERF_MODE_BIT | BMI2_ACC_CONF_BWP_NORMAL_BIT | pIMU->accODR;
    // // Set acc range to pIMU->accRange
    // pBuffer[1] = pIMU->accRange;
    // status = IMUWriteReg(pIMU, BMI2_ACC_CONF_ADDR, pBuffer, 2);

    // /*
    // * Gyro Setup
    // */
    // // enable gyro filter perf bit, set gyr bwp to normal, and set gry_odr to 100 Hz
    // pBuffer[0] = BMI2_GYRO_CONF_FILTER_PERF_BIT | BMI2_GYRO_CONF_NOISE_PERF_BIT | BMI2_GYRO_CONF_BWP_NORMAL_BIT | pIMU->gyroODR;
    // // set gyro range
    // pBuffer[1] = pIMU->gyroRange;
    // status = IMUWriteReg(pIMU, BMI2_GYR_CONF_ADDR, pBuffer, 2);

    // /*
    // * Power Setup
    // */
    // // disable adv power sav and leave fifo self wakeup enabled
    // pBuffer[0] = 0x02;
    // status = IMUWriteReg(pIMU, BMI2_PWR_CONF_ADDR, pBuffer, 1);

    // /*
    // * Interrupt Setup
    // */
    // // enable INT1 w input disabled, output enabled, push pull, and active high
    // pBuffer[0] = (0 << 3) | (1 << 2) | (0 << 1) | (1 << 0);
    // status = IMUWriteReg(pIMU, BMI2_INT1_IO_CTRL_ADDR, pBuffer, 1);
    // // interrupts will NOT be cleared automatically. Have to be cleared by the
    // // the host reading the int status registers
    // pBuffer[0] = 1;
    // status = IMUWriteReg(pIMU, BMI2_INT_LATCH_ADDR, pBuffer, 1);

    return eSTATUS_SUCCESS;
}
