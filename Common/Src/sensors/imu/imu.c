#include "sensors/imu/imu.h"
#include "log.h"
#include "sensors/imu/bmixxx.h"
#include <string.h>

#define BIT_ISSET(v, bit)  ((v & bit) == 1)
#define RW_BUFFER_SZ       16
#define INT_ERR_STATUS_BIT 10

STATUS_TYPE IMUGetErr (IMU* pIMU, IMUErr* pOutErr) {
    uint8_t pBuff[2]      = { 0 };
    STATUS_TYPE status    = IMUReadReg (pIMU, BMI3_REG_ERR_REG, pBuff, 2);
    uint16_t err          = ((uint16_t)pBuff[1] << 8) | (uint16_t)pBuff[0];
    pOutErr->fatalErr     = err & (1 << 0);
    pOutErr->featEngOvrld = err & (1 << 2);
    pOutErr->featEngWd    = err & (1 << 4);
    pOutErr->accConfErr   = err & (1 << 5);
    pOutErr->gyrConfErr   = err & (1 << 6);
    pOutErr->i3cErr0      = err & (1 << 8);
    pOutErr->i3cErr1      = err & (1 << 11);
    if (status != eSTATUS_SUCCESS) {
        return status;
    }
    /* Clear IMU error status on successful read */
    pIMU->status = eSTATUS_SUCCESS;
    return eSTATUS_SUCCESS;
}

void IMULogErr (STATUS_TYPE curImuStatus, IMUErr const* pOutErr) {
    if (curImuStatus == eIMU_COM_FAILURE) {
        LOG_ERROR (
        "IMU Communication failure error. It occurs due to "
        "read/write operation failure and also due to power failure "
        "during communication");
    }
    if (curImuStatus == eIMU_RW_BUFFER_OVERFLOW) {
        LOG_ERROR ("IMU register read or write buffer size was exceeded");
    }
    if (curImuStatus == eIMU_NULL_PTR) {
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
        LOG_ERROR (
        "IMU unsupported accelerometer configuration set by user");
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
        return eIMU_RW_BUFFER_OVERFLOW;
    }

    if (HAL_SPI_TransmitReceive (pIMU->pSPI, pTx, pRx, len + pIMU->nDummyBytes, 100) != HAL_OK) {
        return eIMU_COM_FAILURE;
    }
    // The first nDummyBytes are dummy bytes
    memcpy (pBuf, &pRx[pIMU->nDummyBytes], len);
    return eSTATUS_SUCCESS;
}

STATUS_TYPE IMUWriteReg (IMU const* pIMU, uint8_t reg, uint8_t* pBuf, uint32_t len) {
    uint8_t pTx[RW_BUFFER_SZ] = { 0 };

    if (len + 1 > RW_BUFFER_SZ) {
        return eIMU_RW_BUFFER_OVERFLOW;
    }

    pTx[0] = reg | BMI3_SPI_WR_MASK;
    memcpy (&pTx[1], (void*)pBuf, len);

    if (HAL_SPI_Transmit (pIMU->pSPI, pTx, len + 1, 100) != HAL_OK) {
        return eIMU_COM_FAILURE;
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
    (int32_t)(((uint16_t)pBuffer[1]) << 8) | ((uint16_t)pBuffer[0]);
    pIMU->rawGyro.y =
    (int32_t)(((uint16_t)pBuffer[3]) << 8) | ((uint16_t)pBuffer[2]);
    pIMU->rawGyro.z =
    (int32_t)(((uint16_t)pBuffer[5]) << 8) | ((uint16_t)pBuffer[4]);

    int32_t scale = 125;
    if (pIMU->gyroRange == IMU_GYRO_RANGE_250)
        scale = 250;
    if (pIMU->gyroRange == IMU_GYRO_RANGE_500)
        scale = 500;
    if (pIMU->gyroRange == IMU_GYRO_RANGE_1000)
        scale = 1000;
    if (pIMU->gyroRange == IMU_GYRO_RANGE_2000)
        scale = 2000;
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
    (int32_t)(((uint16_t)pBuffer[1]) << 8) | ((uint16_t)pBuffer[0]);
    pIMU->rawAccel.y =
    (int32_t)(((uint16_t)pBuffer[3]) << 8) | ((uint16_t)pBuffer[2]);
    pIMU->rawAccel.z =
    (int32_t)(((uint16_t)pBuffer[5]) << 8) | ((uint16_t)pBuffer[4]);

    int32_t scale = 2;
    if (pIMU->accRange == IMU_ACC_RANGE_4G)
        scale = 4;
    if (pIMU->accRange == IMU_ACC_RANGE_8G)
        scale = 8;
    if (pIMU->accRange == IMU_ACC_RANGE_16G)
        scale = 16;
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
        return eIMU_NULL_PTR;
    }

    // read both status registers
    uint8_t pBuf[2]    = { 0 };
    STATUS_TYPE status = IMUReadReg (pIMU, BMI3_REG_INT_STATUS_INT1, pBuf, 2);

    if (status != eSTATUS_SUCCESS) {
        return status;
    }

    uint16_t intStatus1 = ((uint16_t)pBuf[1]) << 8 | (uint16_t)pBuf[0];
    /* check if error status bit is set */
    if (BIT_ISSET (intStatus1, (INT_ERR_STATUS_BIT << 1))) {
        return eIMU_HARDWARE_ERR;
    }

    /* check if accel data is ready */
    if (BIT_ISSET (intStatus1, (13 << 1))) {
        status = IMUUpdateAccel (pIMU, *pOutputAccel, pOutputAccel);
        if (status != eSTATUS_SUCCESS) {
            return status;
        }
    }
    /* check if gyro data is ready */
    if (BIT_ISSET (intStatus1, (12 << 1))) {
        status = IMUUpdateGyro (pIMU, *pOutputGyro, pOutputGyro);
        if (status != eSTATUS_SUCCESS) {
            return status;
        }
    }
    /* check if temperature data is ready */
    if (BIT_ISSET (intStatus1, (11 << 1))) {
        if (status != eSTATUS_SUCCESS) {
            return status;
        }
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
                if (regData[0] & BMI3_FEATURE_ENGINE_ENABLE_MASK) {
                    status = eSTATUS_SUCCESS;
                    break;
                }
            }
        }
    }

    return status;
}

STATUS_TYPE IMUSetupInterrupts (IMU const* pIMU) {
    uint8_t pRegData[4] = { 0 };
    uint16_t temp       = 0;
    /* Map all enabled interrupts to pin INT1 */
    uint8_t enable = BMI3_INT1, disable = BMI3_INT_NONE;
    STATUS_TYPE status = IMUReadReg (pIMU, BMI3_REG_INT_MAP1, pRegData, 4);

    if (status == eSTATUS_SUCCESS) {
        pRegData[0] = BMI3_SET_BIT_POS0 (pRegData[0], BMI3_NO_MOTION_OUT, disable);
        pRegData[0] = BMI3_SET_BITS (pRegData[0], BMI3_ANY_MOTION_OUT, disable);
        pRegData[0] = BMI3_SET_BITS (pRegData[0], BMI3_FLAT_OUT, disable);
        pRegData[0] = BMI3_SET_BITS (pRegData[0], BMI3_ORIENTATION_OUT, disable);

        temp = (uint16_t)(pRegData[1]) << 8;
        temp = BMI3_SET_BITS (temp, BMI3_STEP_DETECTOR_OUT, disable);
        temp = BMI3_SET_BITS (temp, BMI3_STEP_COUNTER_OUT, disable);
        temp = BMI3_SET_BITS (temp, BMI3_SIG_MOTION_OUT, disable);
        temp = BMI3_SET_BITS (temp, BMI3_TILT_OUT, disable);
        pRegData[1] = (uint8_t)(temp >> 8);

        pRegData[2] = BMI3_SET_BIT_POS0 (pRegData[2], BMI3_TAP_OUT, disable);
        pRegData[2] = BMI3_SET_BITS (pRegData[2], BMI3_I3C_OUT, disable);
        pRegData[2] = BMI3_SET_BITS (pRegData[2], BMI3_ERR_STATUS, disable);
        pRegData[2] = BMI3_SET_BITS (pRegData[2], BMI3_TEMP_DRDY_INT, enable);

        temp = (uint16_t)(pRegData[3]) << 8;
        temp = BMI3_SET_BITS (temp, BMI3_GYR_DRDY_INT, enable);
        temp = BMI3_SET_BITS (temp, BMI3_ACC_DRDY_INT, enable);
        temp = BMI3_SET_BITS (temp, BMI3_FIFO_WATERMARK_INT, disable);
        temp = BMI3_SET_BITS (temp, BMI3_FIFO_FULL_INT, disable);
        pRegData[3] = (uint8_t)(temp >> 8);

        status = IMUWriteReg (pIMU, BMI3_REG_INT_MAP1, pRegData, 4);
    }

    return status;
}


STATUS_TYPE IMUInit (
IMU* pIMU,
SPI_HandleTypeDef* pSPI,
IMU_ACC_RANGE accRange,
IMU_ACC_ODR accODR,
IMU_GYRO_RANGE gyroRange,
IMU_GYRO_ODR gyroODR) {
    memset (pIMU, 0, sizeof (IMU));
    pIMU->pSPI                 = pSPI;
    pIMU->accRange             = accRange;
    pIMU->accODR               = accODR;
    pIMU->gyroRange            = gyroRange;
    pIMU->gyroODR              = gyroODR;
    pIMU->msLastAccUpdateTime  = HAL_GetTick ();
    pIMU->msLastGyroUpdateTime = HAL_GetTick ();
    /* SPI reads have 1 dummy byte at the beginning */
    pIMU->nDummyBytes = 1;
    pIMU->magic       = IMU_MAGIC;

    STATUS_TYPE status = IMUSoftReset (pIMU);
    if (status != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to soft reset IMU");
        return status;
    }

    uint8_t pChipID[2] = { 0 };
    status             = IMUReadReg (pIMU, BMI3_REG_CHIP_ID, pChipID, 2);
    if (pChipID != BMI323_CHIP_ID) {
        LOG_ERROR ("Failed to find BMI323. Chip ID [%X] is incorrect", pChipID[0]);
        return eSTATUS_FAILURE;
    }

    /* Enable acc, gyro, and temperature - data ready interrupts for pin INT1 */
    status = IMUSetupInterrupts (pIMU);
    if (status != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to setup IMU interrupts");
        return eSTATUS_FAILURE;
    }

    /* Configure Accelerometer */
    {

        uint8_t pRegData[2] = { 0 };
        uint16_t odr, range, bwp, avgNum, accMode;
        odr = BMI3_SET_BIT_POS0 (pRegData[0], BMI3_ACC_ODR, pIMU->accODR);
        /* Set accelerometer range */
        range = BMI3_SET_BITS (pRegData[0], BMI3_ACC_RANGE, pIMU->accRange);
        /* The Accel bandwidth coefficient defines the 3 dB cutoff frequency in relation to the ODR. */
        bwp = BMI3_SET_BITS (pRegData[0], BMI3_ACC_BW, 0x01);
        /* Set accelerometer bandwidth
        Value    Name    Description
        *  0b000     avg_1   No averaging; pass sample without filtering
        *  0b001     avg_2   Averaging of 2 samples
        *  0b010     avg_4   Averaging of 4 samples
        *  0b011     avg_8   Averaging of 8 samples
        *  0b100     avg_16  Averaging of 16 samples
        *  0b101     avg_32  Averaging of 32 samples
        *  0b110     avg_64  Averaging of 64 samples
        */
        avgNum = BMI3_SET_BITS (pRegData[1], BMI3_ACC_AVG_NUM, BMI3_ACC_AVG64);
        /* Enable the accel mode where averaging of samples
         * will be done based on above set bandwidth and ODR.
         * Note : By default accel is disabled. The accel will get enable by selecting the mode.
         */
        accMode = BMI3_SET_BITS (pRegData[1], BMI3_ACC_MODE, BMI3_ACC_MODE_NORMAL);
        pRegData[0] = (uint8_t)(odr | range | bwp);
        pRegData[1] = (uint8_t)((avgNum | accMode) >> 8);
        status      = IMUWriteReg (pIMU, BMI3_REG_ACC_CONF, pRegData, 2);
        if (status != eSTATUS_SUCCESS) {
            LOG_ERROR ("Failed to configure IMU accelerometer");
            return eSTATUS_FAILURE;
        }
    }

    /* Configure Gyro */
    {
        uint8_t pRegData[2] = { 0 };
        uint16_t odr, range, bwp, avgNum, accMode;
        /* Output Data Rate. By default ODR is set as 100Hz for gyro. */
        odr = BMI3_SET_BIT_POS0 (pRegData[0], BMI3_GYR_ODR, pIMU->gyroODR);
        /* Gyroscope Angular Rate Measurement Range. By default the range is 2000dps. */
        range = BMI3_SET_BITS (pRegData[0], BMI3_GYR_RANGE, pIMU->gyroRange);
        /*  The Gyroscope bandwidth coefficient defines the 3 dB cutoff
         * frequency in relation to the ODR Value   Name      Description
         *    0   odr_half   BW = gyr_odr/2
         *    1  odr_quarter BW = gyr_odr/4
         */
        bwp = BMI3_SET_BITS (pRegData[0], BMI3_GYR_BW, BMI3_GYR_BW_ODR_HALF);
        /* Set gyro bandwidth
        Value    Name    Description
        *  0b000     avg_1   No averaging; pass sample without filtering
        *  0b001     avg_2   Averaging of 2 samples
        *  0b010     avg_4   Averaging of 4 samples
        *  0b011     avg_8   Averaging of 8 samples
        *  0b100     avg_16  Averaging of 16 samples
        *  0b101     avg_32  Averaging of 32 samples
        *  0b110     avg_64  Averaging of 64 samples
        */
        avgNum = BMI3_SET_BITS (pRegData[1], BMI3_ACC_AVG_NUM, BMI3_GYR_AVG32);
        /* Enable the gyro mode where averaging of samples
         * will be done based on above set bandwidth and ODR.
         * Note : By default gyro is disabled. The gyro will get enable by selecting the mode.
         */
        accMode = BMI3_SET_BITS (pRegData[1], BMI3_ACC_MODE, BMI3_GYR_MODE_NORMAL);
        pRegData[0] = (uint8_t)(odr | range | bwp);
        pRegData[1] = (uint8_t)((avgNum | accMode) >> 8);
        status      = IMUWriteReg (pIMU, BMI3_REG_GYR_CONF, pRegData, 2);
        if (status != eSTATUS_SUCCESS) {
            LOG_ERROR ("Failed to configure IMU gyroscope");
            return eSTATUS_FAILURE;
        }
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
