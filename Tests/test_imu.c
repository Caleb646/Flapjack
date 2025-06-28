#include "common.h"
#include "hal.h"
#include "sensors/imu/bmixxx.h"
#include "sensors/imu/imu.h"
#include "unity/unity.h"

#include <stdio.h>


#ifndef UNIT_TEST
#error "UNIT_TEST should be defined in this file"
#endif

// // LOG_ERROR stub for host build
// int LOG_ERROR (const char* fmt, ...) {
//     return 0;
// }

uint8_t gIMURegs[256] = { 0 };


HAL_StatusTypeDef
SPITransmitCB (SPI_HandleTypeDef* hspi, uint8_t* pData, uint16_t size, uint32_t timeout) {
    uint8_t reg = pData[0U] & 0x7FU;
    // printf ("pData[0U] = 0x%02X\n", pData[0U]);
    // printf ("reg = 0x%02X\n", reg);
    TEST_ASSERT_TRUE ((pData[0U] & BMI3_SPI_WR_MASK) > 0);
    for (uint16_t i = 1; i < size; ++i) {
        gIMURegs[reg + (i - 1)] = pData[i];
    }
    return HAL_OK;
}

HAL_StatusTypeDef
SPITransmitReceiveCB (SPI_HandleTypeDef* hspi, uint8_t* pTxData, uint8_t* pRxData, uint16_t size, uint32_t timeout) {
    uint8_t reg = pTxData[0U] & 0x7FU;
    TEST_ASSERT_TRUE ((pTxData[0U] & BMI3_SPI_RD_MASK) > 0);
    TEST_ASSERT_TRUE (size > 0);
    for (uint16_t i = 1; i < size; ++i) {
        // 1st byte is a dummy byte
        pRxData[i] = gIMURegs[reg + (i - 1)];
    }
    return HAL_OK;
}

void setUp (void) {
    gIMURegs[BMI3_REG_CHIP_ID]  = BMI323_CHIP_ID;
    gHAL_SPI_Transmit_CB        = SPITransmitCB;
    gHAL_SPI_TransmitReceive_CB = SPITransmitReceiveCB;
}

void tearDown (void) {
}

void test_IMUInit (void) {
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

    gHAL_SPI_Transmit_CB        = SPITransmitCB;
    gHAL_SPI_TransmitReceive_CB = SPITransmitReceiveCB;

    STATUS_TYPE status = IMUInit (&imu, &spi, aconf, gconf);
    TEST_ASSERT_EQUAL_INT (eSTATUS_SUCCESS, status);
    TEST_ASSERT_EQUAL_UINT32 (IMU_MAGIC, imu.magic);
    TEST_ASSERT_EQUAL_PTR (&spi, imu.pSPI);
    TEST_ASSERT_EQUAL (1, imu.nDummyBytes);

    // BMI3_REG_CMD should have soft reset value
    TEST_ASSERT_EQUAL_HEX8 ((BMI3_CMD_SOFT_RESET & 0xFF), gIMURegs[BMI3_REG_CMD]);
    TEST_ASSERT_EQUAL_HEX8 (
    (BMI3_CMD_SOFT_RESET & 0xFF00U) >> 8U, (uint16_t)gIMURegs[BMI3_REG_CMD + 1U]);

    // Check feature engine enable (written in IMUSoftReset)
    TEST_ASSERT_EQUAL_HEX8 (0x2C, gIMURegs[BMI3_REG_FEATURE_IO2]);
    TEST_ASSERT_EQUAL_HEX8 (0x01, gIMURegs[BMI3_REG_FEATURE_IO2 + 1U]);
    TEST_ASSERT_EQUAL_HEX8 (BMI3_ENABLE, gIMURegs[BMI3_REG_FEATURE_IO_STATUS]);
    TEST_ASSERT_EQUAL_HEX8 (BMI3_ENABLE, gIMURegs[BMI3_REG_FEATURE_CTRL]);

    // Check accel/gyro config (IMUSetConf)
    uint8_t expected1stAccConf = aconf.odr | (aconf.range << 4) | (aconf.bw << 7);
    uint8_t expected2ndAccConf = aconf.avg | (aconf.mode << 4);
    TEST_ASSERT_EQUAL_HEX8 (expected1stAccConf, gIMURegs[BMI3_REG_ACC_CONF]);
    TEST_ASSERT_EQUAL_HEX8 (expected2ndAccConf, gIMURegs[BMI3_REG_ACC_CONF + 1U]);

    uint8_t expected1stGyroConf = gconf.odr | (gconf.range << 4) | (gconf.bw << 7);
    uint8_t expected2ndGyroConf = gconf.avg | (gconf.mode << 4);
    TEST_ASSERT_EQUAL_HEX8 (expected1stGyroConf, gIMURegs[BMI3_REG_GYR_CONF]);
    TEST_ASSERT_EQUAL_HEX8 (expected2ndGyroConf, gIMURegs[BMI3_REG_GYR_CONF + 1U]);

    // Check interrupts (IMUSetupInterrupts)
    TEST_ASSERT_TRUE (gIMURegs[BMI3_REG_INT_MAP1] == 0);
    TEST_ASSERT_TRUE (gIMURegs[BMI3_REG_INT_MAP1 + 1] == 0);
    TEST_ASSERT_TRUE (gIMURegs[BMI3_REG_INT_MAP1 + 2] == (1 << 6));
    TEST_ASSERT_TRUE (gIMURegs[BMI3_REG_INT_MAP1 + 3] == (1 | (1 << 2)));
}


int main (void) {
    UNITY_BEGIN ();
    RUN_TEST (test_IMUInit);
    return UNITY_END ();
}
