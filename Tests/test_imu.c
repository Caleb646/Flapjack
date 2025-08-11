#include "common.h"
#include "hal.h"
#include "sensors/imu/bmixxx.h"
#include "sensors/imu/imu.h"
#include "unity/unity.h"
#include <math.h>

#include <stdio.h>

#ifndef UNIT_TEST
#error "UNIT_TEST should be defined in this file"
#endif


uint16_t gIMURegs[256] = { 0 };

HAL_StatusTypeDef
SPITransmitCB (SPI_HandleTypeDef* hspi, uint8_t* pData, uint16_t size, uint32_t timeout) {
    uint8_t reg = pData[0U] & 0x7FU;
    // printf ("pData[0U] = 0x%02X\n", pData[0U]);
    // printf ("reg = 0x%02X\n", reg);
    TEST_ASSERT_TRUE ((pData[0U] & BMI3_SPI_WR_MASK) > 0U);
    uint16_t regIdx = 0;
    for (uint16_t i = 1; i < size; i += 2) {
        if (i + 1 < size) {
            gIMURegs[reg + regIdx++] = ((uint16_t)pData[i + 1U] << 8U) | pData[i];
        } else {
            gIMURegs[reg + regIdx++] = pData[i];
        }
    }

    if (reg == BMI3_REG_FEATURE_CTRL && pData[1U] & BMI3_ENABLE) {
        gIMURegs[BMI3_REG_FEATURE_IO1] = BMI3_ENABLE;
    }

    if (reg == BMI3_REG_CMD) {
        uint16_t cmd = ((uint16_t)pData[2U] << 8U) | pData[1U];
        if (cmd == BMI3_CMD_SELF_CALIB_TRIGGER) {
            gIMURegs[BMI3_REG_FEATURE_IO1] = (1U << 11U);
            // Set error status bit to indicate calibration finished
            gIMURegs[BMI3_REG_INT_STATUS_INT1] = (1U << 10U);
        }
    }

    return HAL_OK;
}

HAL_StatusTypeDef
SPITransmitReceiveCB (SPI_HandleTypeDef* hspi, uint8_t* pTxData, uint8_t* pRxData, uint16_t size, uint32_t timeout) {
    uint8_t reg = pTxData[0U] & 0x7FU;
    TEST_ASSERT_TRUE ((pTxData[0U] & BMI3_SPI_RD_MASK) > 0);
    TEST_ASSERT_TRUE (size > 1);

    /*
     * If we are reading from feature IO1 register and are in
     * self calibration mode, 0x1, change system state back to normal 0x0
     */
    if (reg == BMI3_REG_FEATURE_IO1 && gIMURegs[BMI3_REG_FEATURE_IO1] & (1U << 11U)) {
        // The self calibration finished flag has been read so we can set
        // the system state back to normal the error condition to no error.
        if ((gIMURegs[BMI3_REG_INT_STATUS_INT1] & (1U << 10U)) == FALSE) {
            gIMURegs[BMI3_REG_FEATURE_IO1] |= (1U << 5U) | (1U << 4U) | (5U << 0U);
            gIMURegs[BMI3_REG_FEATURE_IO1] &= ~(3U << 11U);
        }
    }

    if (reg == BMI3_REG_INT_STATUS_INT1 && gIMURegs[BMI3_REG_FEATURE_IO1] & (1U << 11U)) {
        gIMURegs[BMI3_REG_INT_STATUS_INT1] &= ~(1U << 10U);
    }

    /*
     * NOTE: Position        Sent        Received
     *       0               addr         0x00
     *       1               0x00         dummy byte (0x00)
     *       2               0x00         data byte 0
     *       N               0x00         data byte N - 1
     */
    uint16_t regIdx = 0;
    for (uint16_t i = 2; i < size; i += 2) {
        if (i + 1 < size) {
            pRxData[i]     = gIMURegs[reg + regIdx];
            pRxData[i + 1] = gIMURegs[reg + regIdx++] >> 8U;
        } else {
            pRxData[i] = gIMURegs[reg + regIdx++];
        }
    }

    return HAL_OK;
}

void setUpIMU (void) {
    memset (gIMURegs, 0, sizeof (gIMURegs));
    gIMURegs[BMI3_REG_CHIP_ID] = BMI323_CHIP_ID;
}

// void tearDown (void) {
// }

void test_IMUInit (void) {
    setUpIMU ();
    gHAL_SPI_Transmit_CB        = SPITransmitCB;
    gHAL_SPI_TransmitReceive_CB = SPITransmitReceiveCB;

    IMU imu;
    SPI_HandleTypeDef spi;
    IMUAccConf aconf  = { 0 };
    aconf.odr         = eIMU_ACC_ODR_100;
    aconf.range       = eIMU_ACC_RANGE_4G;
    aconf.avg         = eIMU_ACC_AVG_32;
    aconf.bw          = eIMU_ACC_BW_HALF;
    aconf.mode        = eIMU_ACC_MODE_HIGH_PERF;
    IMUGyroConf gconf = { 0 };
    gconf.odr         = eIMU_GYRO_ODR_100;
    gconf.range       = eIMU_GYRO_RANGE_250;
    gconf.avg         = eIMU_GYRO_AVG_16;
    gconf.bw          = eIMU_GYRO_BW_HALF;
    gconf.mode        = eIMU_GYRO_MODE_HIGH_PERF;

    eSTATUS_t status = IMUInit (&imu, &spi, NULL);
    TEST_ASSERT_EQUAL_INT (eSTATUS_SUCCESS, status);
    TEST_ASSERT_EQUAL_PTR (&spi, imu.pSPI);
    TEST_ASSERT_EQUAL (1U, imu.nDummyBytes);

    // BMI3_CMD_SELF_CALIB_TRIGGER should have self calibration value
    TEST_ASSERT_EQUAL_HEX16 (BMI3_CMD_SELF_CALIB_TRIGGER, gIMURegs[BMI3_REG_CMD]);
    // Check feature engine enable (written in IMUSoftReset)
    TEST_ASSERT_EQUAL_HEX16 ((0x01U << 8U) | 0x2CU, gIMURegs[BMI3_REG_FEATURE_IO2]);
    TEST_ASSERT_EQUAL_HEX16 (BMI3_ENABLE, gIMURegs[BMI3_REG_FEATURE_IO_STATUS]);
    TEST_ASSERT_EQUAL_HEX16 (BMI3_ENABLE, gIMURegs[BMI3_REG_FEATURE_CTRL]);

    // Check accel/gyro config (IMUSetConf)
    IMUAccConf expectedAccConf;
    IMUGyroConf expectedGyroConf;
    status = IMUGetConf (&imu, &expectedAccConf, &expectedGyroConf);
    TEST_ASSERT_EQUAL_INT (eSTATUS_SUCCESS, status);
    TEST_ASSERT_EQUAL_CHAR_ARRAY (&expectedAccConf, &aconf, sizeof (IMUAccConf));
    TEST_ASSERT_EQUAL_CHAR_ARRAY (&expectedAccConf, &imu.aconf, sizeof (IMUAccConf));
    TEST_ASSERT_EQUAL_CHAR_ARRAY (&expectedGyroConf, &gconf, sizeof (IMUGyroConf));
    TEST_ASSERT_EQUAL_CHAR_ARRAY (&expectedGyroConf, &imu.gconf, sizeof (IMUGyroConf));

    // Check interrupts (IMUSetupInterrupts)
    TEST_ASSERT_TRUE (gIMURegs[BMI3_REG_INT_MAP1] == 0);

    uint16_t temp = 0;
    temp          = BMI3_SET_BITS (temp, BMI3_TEMP_DRDY_INT, 1U);
    temp          = BMI3_SET_BITS (temp, BMI3_GYR_DRDY_INT, 1U);
    temp          = BMI3_SET_BITS (temp, BMI3_ACC_DRDY_INT, 1U);
    TEST_ASSERT_EQUAL_HEX16 (temp, gIMURegs[BMI3_REG_INT_MAP1 + 1]);

    status = IMUEnableInterrupts (&imu);
    TEST_ASSERT_EQUAL_INT (eSTATUS_SUCCESS, status);
    // Check that imu interrupts are enabled
    TEST_ASSERT_EQUAL_HEX16 (
    ((1U << 2U | 1U << 0U) << 8U) | (1U << 2U | 1U << 0U),
    gIMURegs[BMI3_REG_IO_INT_CTRL]);
}

void test_IMUConf (void) {
    setUpIMU ();
    gHAL_SPI_Transmit_CB        = SPITransmitCB;
    gHAL_SPI_TransmitReceive_CB = SPITransmitReceiveCB;

    eSTATUS_t status = eSTATUS_SUCCESS;
    IMU imu          = { .nDummyBytes = 1U };
    {
        IMUAccConf aconf  = { 0 };
        aconf.odr         = eIMU_ACC_ODR_100;
        aconf.range       = eIMU_ACC_RANGE_4G;
        aconf.avg         = eIMU_ACC_AVG_32;
        aconf.bw          = eIMU_GYRO_BW_QUARTER;
        aconf.mode        = eIMU_ACC_MODE_HIGH_PERF;
        IMUGyroConf gconf = { 0 };
        gconf.odr         = eIMU_GYRO_ODR_100;
        gconf.range       = eIMU_GYRO_RANGE_250;
        gconf.avg         = eIMU_GYRO_AVG_16;
        gconf.bw          = eIMU_GYRO_BW_QUARTER;
        gconf.mode        = eIMU_GYRO_MODE_HIGH_PERF;
        status            = IMUSetConf (&imu, &aconf, &gconf);
        TEST_ASSERT_EQUAL_INT (eSTATUS_SUCCESS, status);
        TEST_ASSERT_EQUAL_CHAR_ARRAY (&aconf, &imu.aconf, sizeof (IMUAccConf));
        TEST_ASSERT_EQUAL_CHAR_ARRAY (&gconf, &imu.gconf, sizeof (IMUGyroConf));

        IMUAccConf expectedAccConf;
        IMUGyroConf expectedGyroConf;
        status = IMUGetConf (&imu, &expectedAccConf, &expectedGyroConf);
        TEST_ASSERT_EQUAL_INT (eSTATUS_SUCCESS, status);
        TEST_ASSERT_EQUAL_CHAR_ARRAY (&expectedAccConf, &aconf, sizeof (IMUAccConf));
        TEST_ASSERT_EQUAL_CHAR_ARRAY (&expectedGyroConf, &gconf, sizeof (IMUGyroConf));

        status = IMUCompareConfs (aconf, gconf, aconf, gconf);
        TEST_ASSERT_EQUAL_INT (eSTATUS_SUCCESS, status);
    }
    {
        IMUAccConf aIMUConf  = { 0 };
        aIMUConf.odr         = eIMU_ACC_ODR_50;
        aIMUConf.range       = eIMU_ACC_RANGE_4G;
        aIMUConf.avg         = eIMU_ACC_AVG_8;
        aIMUConf.bw          = eIMU_GYRO_BW_QUARTER;
        aIMUConf.mode        = eIMU_ACC_MODE_HIGH_PERF;
        IMUGyroConf gIMUConf = { 0 };
        gIMUConf.odr         = eIMU_GYRO_ODR_50;
        gIMUConf.range       = eIMU_GYRO_RANGE_250;
        gIMUConf.avg         = eIMU_GYRO_AVG_8;
        gIMUConf.bw          = eIMU_GYRO_BW_QUARTER;
        gIMUConf.mode        = eIMU_GYRO_MODE_HIGH_PERF;

        IMUAccConf aconf  = { 0 };
        aconf.odr         = eIMU_ACC_ODR_100;
        aconf.range       = eIMU_ACC_RANGE_4G;
        aconf.avg         = eIMU_ACC_AVG_32;
        aconf.mode        = eIMU_ACC_MODE_HIGH_PERF;
        IMUGyroConf gconf = { 0 };
        gconf.odr         = eIMU_GYRO_ODR_100;
        gconf.range       = eIMU_GYRO_RANGE_250;
        gconf.avg         = eIMU_GYRO_AVG_16;
        gconf.mode        = eIMU_GYRO_MODE_HIGH_PERF;

        status = IMUSetConf (&imu, &aIMUConf, &gIMUConf);
        TEST_ASSERT_EQUAL_INT (eSTATUS_SUCCESS, status);

        status = IMUSetAltConf (&imu, &aconf, &gconf);
        TEST_ASSERT_EQUAL_INT (eSTATUS_SUCCESS, status);

        IMUAccConf aconf2  = { 0 };
        IMUGyroConf gconf2 = { 0 };
        status             = IMUGetAltConf (&imu, &aconf2, &gconf2);
        TEST_ASSERT_EQUAL_INT (eSTATUS_SUCCESS, status);
        TEST_ASSERT_EQUAL_CHAR_ARRAY (&aconf, &aconf2, sizeof (IMUAccConf));
        TEST_ASSERT_EQUAL_CHAR_ARRAY (&gconf, &gconf2, sizeof (IMUGyroConf));
        /*
         * Make sure that the alt config doesn't get set on the imu
         */
        TEST_ASSERT_NOT_EQUAL_INT (
        eSTATUS_SUCCESS, IMUCompareConfs (aconf, gconf, imu.aconf, imu.gconf));
    }
}

void test_IMUUpdate (void) {
    eSTATUS_t status = eSTATUS_SUCCESS;
    IMU imu;
    SPI_HandleTypeDef spi;

    // Initialize IMU with basic configuration
    IMUAccConf aconf  = { 0 };
    aconf.odr         = eIMU_ACC_ODR_100;
    aconf.range       = eIMU_ACC_RANGE_4G;
    aconf.avg         = eIMU_ACC_AVG_32;
    aconf.bw          = eIMU_ACC_BW_HALF;
    aconf.mode        = eIMU_ACC_MODE_HIGH_PERF;
    IMUGyroConf gconf = { 0 };
    gconf.odr         = eIMU_GYRO_ODR_100;
    gconf.range       = eIMU_GYRO_RANGE_250;
    gconf.avg         = eIMU_GYRO_AVG_16;
    gconf.bw          = eIMU_GYRO_BW_HALF;
    gconf.mode        = eIMU_GYRO_MODE_HIGH_PERF;

    setUpIMU ();
    status = IMUInit (&imu, &spi, NULL);
    TEST_ASSERT_EQUAL_INT (eSTATUS_SUCCESS, status);

    // Set up test data in simulated registers
    // Accelerometer data (X, Y, Z) - 16-bit values
    gIMURegs[BMI3_REG_ACC_DATA_X] = 0x1234; // X-axis accel data
    gIMURegs[BMI3_REG_ACC_DATA_Y] = 0x5678; // Y-axis accel data
    gIMURegs[BMI3_REG_ACC_DATA_Z] = 0x9ABC; // Z-axis accel data

    // Gyroscope data (X, Y, Z) - 16-bit values
    gIMURegs[BMI3_REG_GYR_DATA_X] = 0xDEF0; // X-axis gyro data
    gIMURegs[BMI3_REG_GYR_DATA_Y] = 0x1357; // Y-axis gyro data
    gIMURegs[BMI3_REG_GYR_DATA_Z] = 0x2468; // Z-axis gyro data

    // Test IMUUpdateAccel
    status = IMUUpdateAccel (&imu);
    TEST_ASSERT_EQUAL_INT (eSTATUS_SUCCESS, status);

    // Verify accelerometer data was read correctly
    // The IMUUpdateAccel function reads into rawAccel.x, .y, .z
    TEST_ASSERT_EQUAL_HEX16 (0x1234, imu.rawAccel.x);
    TEST_ASSERT_EQUAL_HEX16 (0x5678, imu.rawAccel.y);
    TEST_ASSERT_EQUAL_HEX16 (0x9ABC, imu.rawAccel.z);

    // Test IMUUpdateGyro
    status = IMUUpdateGyro (&imu);
    TEST_ASSERT_EQUAL_INT (eSTATUS_SUCCESS, status);

    // Verify gyroscope data was read correctly
    TEST_ASSERT_EQUAL_HEX16 (0xDEF0, imu.rawGyro.x);
    TEST_ASSERT_EQUAL_HEX16 (0x1357, imu.rawGyro.y);
    TEST_ASSERT_EQUAL_HEX16 (0x2468, imu.rawGyro.z);

    // Test with different data to ensure updates work
    gIMURegs[BMI3_REG_ACC_DATA_X] = 0x0F0F;
    gIMURegs[BMI3_REG_ACC_DATA_Y] = 0xF0F0;
    gIMURegs[BMI3_REG_ACC_DATA_Z] = 0xAAAA;

    int32_t prevAX = imu.rawAccel.x;
    int32_t prevAY = imu.rawAccel.y;
    int32_t prevAZ = imu.rawAccel.z;

    status = IMUUpdateAccel (&imu);
    TEST_ASSERT_EQUAL_INT (eSTATUS_SUCCESS, status);

    // Verify the values changed (assuming the conversion produces different results)
    // This test ensures the update is actually reading new data
    TEST_ASSERT_TRUE (
    imu.rawAccel.x != prevAX || imu.rawAccel.y != prevAY || imu.rawAccel.z != prevAZ);

    // Test with different gyro data
    gIMURegs[BMI3_REG_GYR_DATA_X] = 0x5555;
    gIMURegs[BMI3_REG_GYR_DATA_Y] = 0x3333;
    gIMURegs[BMI3_REG_GYR_DATA_Z] = 0x7777;

    int32_t prevGX = imu.rawGyro.x;
    int32_t prevGY = imu.rawGyro.y;
    int32_t prevGZ = imu.rawGyro.z;

    status = IMUUpdateGyro (&imu);
    TEST_ASSERT_EQUAL_INT (eSTATUS_SUCCESS, status);

    // Verify the gyro values changed
    TEST_ASSERT_TRUE (
    imu.rawGyro.x != prevGX || imu.rawGyro.y != prevGY || imu.rawGyro.z != prevGZ);

    // Test IMUConvertRaw function
    // Set up known raw values for conversion testing
    Vec3 testRawAccel    = { .x = 1000, .y = -2000, .z = 3000 };
    Vec3 testRawGyro     = { .x = 500, .y = -1000, .z = 1500 };
    Vec3f convertedAccel = { 0 };
    Vec3f convertedGyro  = { 0 };

    // Test conversion with 4G accel range and 250 dps gyro range
    status = IMUConvertRaw (
    eIMU_ACC_RANGE_4G, testRawAccel, eIMU_GYRO_RANGE_250, testRawGyro,
    &convertedAccel, &convertedGyro);
    TEST_ASSERT_EQUAL_INT (eSTATUS_SUCCESS, status);

    // Verify that conversion produces reasonable values
    // For accelerometer: should be in m/s² (typical range -40 to +40 m/s² for 4G)
    TEST_ASSERT_TRUE (convertedAccel.x > -50.0F && convertedAccel.x < 50.0F);
    TEST_ASSERT_TRUE (convertedAccel.y > -50.0F && convertedAccel.y < 50.0F);
    TEST_ASSERT_TRUE (convertedAccel.z > -50.0F && convertedAccel.z < 50.0F);

    // For gyroscope: should be in degrees/second (typical range -250 to +250 dps)
    TEST_ASSERT_TRUE (convertedGyro.x > -300.0F && convertedGyro.x < 300.0F);
    TEST_ASSERT_TRUE (convertedGyro.y > -300.0F && convertedGyro.y < 300.0F);
    TEST_ASSERT_TRUE (convertedGyro.z > -300.0F && convertedGyro.z < 300.0F);

    // Test with different ranges to ensure scaling works correctly
    Vec3f convertedAccel2G = { 0 };
    Vec3f convertedGyro125 = { 0 };

    status = IMUConvertRaw (
    eIMU_ACC_RANGE_2G, testRawAccel, eIMU_GYRO_RANGE_125, testRawGyro,
    &convertedAccel2G, &convertedGyro125);
    TEST_ASSERT_EQUAL_INT (eSTATUS_SUCCESS, status);

    // 2G range should give smaller acceleration values than 4G for same raw data
    TEST_ASSERT_TRUE (fabs (convertedAccel2G.x) < fabs (convertedAccel.x));
    TEST_ASSERT_TRUE (fabs (convertedAccel2G.y) < fabs (convertedAccel.y));
    TEST_ASSERT_TRUE (fabs (convertedAccel2G.z) < fabs (convertedAccel.z));

    // 125 dps range should give smaller angular velocity values than 250 dps
    TEST_ASSERT_TRUE (fabs (convertedGyro125.x) < fabs (convertedGyro.x));
    TEST_ASSERT_TRUE (fabs (convertedGyro125.y) < fabs (convertedGyro.y));
    TEST_ASSERT_TRUE (fabs (convertedGyro125.z) < fabs (convertedGyro.z));

    // Test with zero raw values
    Vec3 zeroRaw    = { .x = 0, .y = 0, .z = 0 };
    Vec3f zeroAccel = { 0 };
    Vec3f zeroGyro  = { 0 };

    status =
    IMUConvertRaw (eIMU_ACC_RANGE_4G, zeroRaw, eIMU_GYRO_RANGE_250, zeroRaw, &zeroAccel, &zeroGyro);
    TEST_ASSERT_EQUAL_INT (eSTATUS_SUCCESS, status);

    // Zero raw values should produce zero or near-zero converted values
    TEST_ASSERT_TRUE (fabs (zeroAccel.x) < 0.1F);
    TEST_ASSERT_TRUE (fabs (zeroAccel.y) < 0.1F);
    TEST_ASSERT_TRUE (fabs (zeroAccel.z) < 0.1F);
    TEST_ASSERT_TRUE (fabs (zeroGyro.x) < 0.1F);
    TEST_ASSERT_TRUE (fabs (zeroGyro.y) < 0.1F);
    TEST_ASSERT_TRUE (fabs (zeroGyro.z) < 0.1F);

    // Test with maximum positive and negative values
    Vec3 maxRaw    = { .x = 32767, .y = -32768, .z = 32767 };
    Vec3f maxAccel = { 0 };
    Vec3f maxGyro  = { 0 };

    status =
    IMUConvertRaw (eIMU_ACC_RANGE_16G, maxRaw, eIMU_GYRO_RANGE_2000, maxRaw, &maxAccel, &maxGyro);
    TEST_ASSERT_EQUAL_INT (eSTATUS_SUCCESS, status);

    // Maximum values should be within the expected range limits
    // 16G = ~157 m/s², 2000 dps should be close to limits
    TEST_ASSERT_TRUE (fabs (maxAccel.x) < 200.0F); // Should be close to ~157 m/s²
    TEST_ASSERT_TRUE (fabs (maxAccel.y) < 200.0F);
    TEST_ASSERT_TRUE (fabs (maxAccel.z) < 200.0F);
    TEST_ASSERT_TRUE (fabs (maxGyro.x) < 2100.0F); // Should be close to 2000 dps
    TEST_ASSERT_TRUE (fabs (maxGyro.y) < 2100.0F);
    TEST_ASSERT_TRUE (fabs (maxGyro.z) < 2100.0F);

    // Test exact conversion values
    // Testing with known raw values to verify exact conversion math

    // Test 1: Simple positive values with 2G accel range and 125 dps gyro range
    Vec3 exactRaw1 = { .x = 16384, .y = 8192, .z = -16384 }; // Half-scale values
    Vec3f exactAccel1 = { 0 };
    Vec3f exactGyro1  = { 0 };

    status =
    IMUConvertRaw (eIMU_ACC_RANGE_2G, exactRaw1, eIMU_GYRO_RANGE_125, exactRaw1, &exactAccel1, &exactGyro1);
    TEST_ASSERT_EQUAL_INT (eSTATUS_SUCCESS, status);

    // For 2G range: 2G * 9.81 m/s² = 19.62 m/s² full scale
    // 16384 / 32768 = 0.5, so expect ~9.81 m/s²
    TEST_ASSERT_FLOAT_WITHIN (1.0F, 9.81F, exactAccel1.x);
    TEST_ASSERT_FLOAT_WITHIN (0.5F, 4.905F, exactAccel1.y); // Half of above
    TEST_ASSERT_FLOAT_WITHIN (1.0F, -9.81F, exactAccel1.z); // Negative

    // For 125 dps range: 16384 / 32768 = 0.5, so expect 62.5 dps
    TEST_ASSERT_FLOAT_WITHIN (5.0F, 62.5F, exactGyro1.x);
    TEST_ASSERT_FLOAT_WITHIN (2.5F, 31.25F, exactGyro1.y); // Half of above
    TEST_ASSERT_FLOAT_WITHIN (5.0F, -62.5F, exactGyro1.z); // Negative

    // Test 2: Quarter-scale values with 4G accel and 250 dps gyro
    Vec3 exactRaw2 = { .x = 8192, .y = -8192, .z = 4096 }; // Quarter and eighth scale
    Vec3f exactAccel2 = { 0 };
    Vec3f exactGyro2  = { 0 };

    status =
    IMUConvertRaw (eIMU_ACC_RANGE_4G, exactRaw2, eIMU_GYRO_RANGE_250, exactRaw2, &exactAccel2, &exactGyro2);
    TEST_ASSERT_EQUAL_INT (eSTATUS_SUCCESS, status);

    // For 4G range: 4G * 9.81 m/s² = 39.24 m/s² full scale
    // 8192 / 32768 = 0.25, so expect ~9.81 m/s²
    TEST_ASSERT_FLOAT_WITHIN (1.0F, 9.81F, exactAccel2.x);
    TEST_ASSERT_FLOAT_WITHIN (1.0F, -9.81F, exactAccel2.y);
    TEST_ASSERT_FLOAT_WITHIN (0.5F, 4.905F, exactAccel2.z); // Eighth scale

    // For 250 dps range: 8192 / 32768 = 0.25, so expect 62.5 dps
    TEST_ASSERT_FLOAT_WITHIN (5.0F, 62.5F, exactGyro2.x);
    TEST_ASSERT_FLOAT_WITHIN (5.0F, -62.5F, exactGyro2.y);
    TEST_ASSERT_FLOAT_WITHIN (2.5F, 31.25F, exactGyro2.z);

    // Test 3: Full-scale values to verify maximum conversion
    Vec3 fullScaleRaw = { .x = 32767, .y = -32768, .z = 16384 };
    Vec3f fullAccel   = { 0 };
    Vec3f fullGyro    = { 0 };

    status = IMUConvertRaw (
    eIMU_ACC_RANGE_8G, fullScaleRaw, eIMU_GYRO_RANGE_500, fullScaleRaw,
    &fullAccel, &fullGyro);
    TEST_ASSERT_EQUAL_INT (eSTATUS_SUCCESS, status);

    // For 8G range: 8G * 9.81 m/s² = 78.48 m/s² full scale
    // 32767 / 32768 ≈ 1.0, so expect ~78.48 m/s²
    TEST_ASSERT_FLOAT_WITHIN (2.0F, 78.48F, fullAccel.x);
    TEST_ASSERT_FLOAT_WITHIN (2.0F, -78.48F, fullAccel.y);
    TEST_ASSERT_FLOAT_WITHIN (1.0F, 39.24F, fullAccel.z); // Half scale

    // For 500 dps range: 32767 / 32768 ≈ 1.0, so expect ~500 dps
    TEST_ASSERT_FLOAT_WITHIN (10.0F, 500.0F, fullGyro.x);
    TEST_ASSERT_FLOAT_WITHIN (10.0F, -500.0F, fullGyro.y);
    TEST_ASSERT_FLOAT_WITHIN (5.0F, 250.0F, fullGyro.z); // Half scale

    // Test 4: Small values to test precision
    Vec3 smallRaw    = { .x = 100, .y = -200, .z = 50 };
    Vec3f smallAccel = { 0 };
    Vec3f smallGyro  = { 0 };

    status =
    IMUConvertRaw (eIMU_ACC_RANGE_2G, smallRaw, eIMU_GYRO_RANGE_125, smallRaw, &smallAccel, &smallGyro);
    TEST_ASSERT_EQUAL_INT (eSTATUS_SUCCESS, status);

    // For small values, ensure they're proportionally converted
    // 100 / 32768 * 19.62 m/s² ≈ 0.0599 m/s²
    TEST_ASSERT_FLOAT_WITHIN (0.01F, 0.0599F, smallAccel.x);
    TEST_ASSERT_FLOAT_WITHIN (0.01F, -0.1198F, smallAccel.y);  // Double
    TEST_ASSERT_FLOAT_WITHIN (0.005F, 0.02995F, smallAccel.z); // Half

    // 100 / 32768 * 125 dps ≈ 0.381 dps
    TEST_ASSERT_FLOAT_WITHIN (0.05F, 0.381F, smallGyro.x);
    TEST_ASSERT_FLOAT_WITHIN (0.05F, -0.763F, smallGyro.y); // Double
    TEST_ASSERT_FLOAT_WITHIN (0.025F, 0.191F, smallGyro.z); // Half

    // Test 5: Edge case - single LSB values
    Vec3 lsbRaw    = { .x = 1, .y = -1, .z = 2 };
    Vec3f lsbAccel = { 0 };
    Vec3f lsbGyro  = { 0 };

    status =
    IMUConvertRaw (eIMU_ACC_RANGE_16G, lsbRaw, eIMU_GYRO_RANGE_2000, lsbRaw, &lsbAccel, &lsbGyro);
    TEST_ASSERT_EQUAL_INT (eSTATUS_SUCCESS, status);

    // For 16G range: 1 LSB = 16G * 9.81 / 32768 ≈ 0.00478 m/s²
    TEST_ASSERT_FLOAT_WITHIN (0.001F, 0.00478F, lsbAccel.x);
    TEST_ASSERT_FLOAT_WITHIN (0.001F, -0.00478F, lsbAccel.y);
    TEST_ASSERT_FLOAT_WITHIN (0.001F, 0.00956F, lsbAccel.z); // 2 LSB

    // For 2000 dps range: 1 LSB = 2000 / 32768 ≈ 0.061 dps
    TEST_ASSERT_FLOAT_WITHIN (0.01F, 0.061F, lsbGyro.x);
    TEST_ASSERT_FLOAT_WITHIN (0.01F, -0.061F, lsbGyro.y);
    TEST_ASSERT_FLOAT_WITHIN (0.01F, 0.122F, lsbGyro.z); // 2 LSB
}

void test_IMUSelfCalibrate (void) {
    setUpIMU ();
    gHAL_SPI_Transmit_CB        = SPITransmitCB;
    gHAL_SPI_TransmitReceive_CB = SPITransmitReceiveCB;

    IMU imu;
    SPI_HandleTypeDef spi;
    IMUAccConf aconf  = { 0 };
    aconf.odr         = eIMU_ACC_ODR_100;
    aconf.range       = eIMU_ACC_RANGE_4G;
    aconf.avg         = eIMU_ACC_AVG_32;
    aconf.bw          = eIMU_ACC_BW_HALF;
    aconf.mode        = eIMU_ACC_MODE_HIGH_PERF;
    IMUGyroConf gconf = { 0 };
    gconf.odr         = eIMU_GYRO_ODR_100;
    gconf.range       = eIMU_GYRO_RANGE_250;
    gconf.avg         = eIMU_GYRO_AVG_16;
    gconf.bw          = eIMU_GYRO_BW_HALF;
    gconf.mode        = eIMU_GYRO_MODE_HIGH_PERF;

    eSTATUS_t status = IMUInit (&imu, &spi, NULL);
    TEST_ASSERT_EQUAL_INT (eSTATUS_SUCCESS, status);

    // Test successful self-calibration
    IMUSelfCalibResult calibResult = { 0 };

    status =
    IMUCalibrate (&imu, BMI3_SC_SENSITIVITY_EN | BMI3_SC_OFFSET_EN, BMI3_SC_APPLY_CORR_EN, &calibResult);

    TEST_ASSERT_EQUAL_INT (eSTATUS_SUCCESS, status);
    TEST_ASSERT_TRUE (calibResult.result);
    TEST_ASSERT_EQUAL_INT (0, calibResult.error);

    // // Test failed self-calibration
    // IMUSelfCalibResult calibResult2 = { 0 };

    // // Set up the mock behavior for failed calibration
    // setupFailedSelfCalibration ();

    // status =
    // IMUCalibrate (&imu, BMI3_SC_SENSITIVITY_EN | BMI3_SC_OFFSET_EN, BMI3_SC_APPLY_CORR_EN, &calibResult2);

    // TEST_ASSERT_EQUAL_INT (eSTATUS_SUCCESS, status); // Function returns success but result indicates failure
    // TEST_ASSERT_FALSE (calibResult2.result);
    // TEST_ASSERT_NOT_EQUAL_INT (0, calibResult2.error);
}
