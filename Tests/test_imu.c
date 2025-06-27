
#include "unity/unity.h"
#include "sensors/imu/imu.h"
#include "common.h"
#include "hal.h"

#ifndef UNIT_TEST
#error "UNIT_TEST should not be defined in this file"
#endif

// LOG_ERROR stub for host build
int LOG_ERROR(const char* fmt, ...) { return 0; }

void setUp(void) {}
void tearDown(void) {}

void test_IMUInit_ShouldSetMagicAndReturnSuccess(void) {
    IMU imu;
    SPI_HandleTypeDef spi;
    IMUAccConf aconf = {0};
    IMUGyroConf gconf = {0};
    STATUS_TYPE status = IMUInit(&imu, &spi, aconf, gconf);
    TEST_ASSERT_EQUAL_INT(eSTATUS_SUCCESS, status); // s
    TEST_ASSERT_EQUAL_UINT32(IMU_MAGIC, imu.magic);
}

void test_IMUGetErr_ShouldReturnSuccess(void) {
    IMU imu = {0};
    IMUErr err = {0};
    STATUS_TYPE status = IMUGetErr(&imu, &err);
    TEST_ASSERT_EQUAL_INT(eSTATUS_SUCCESS, status);
}

void test_IMUSetConf_ShouldReturnSuccess(void) {
    IMU imu = {0};
    IMUAccConf aconf = {0};
    IMUGyroConf gconf = {0};
    STATUS_TYPE status = IMUSetConf(&imu, &aconf, &gconf);
    TEST_ASSERT_EQUAL_INT(eSTATUS_SUCCESS, status);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_IMUInit_ShouldSetMagicAndReturnSuccess);
    RUN_TEST(test_IMUGetErr_ShouldReturnSuccess);
    RUN_TEST(test_IMUSetConf_ShouldReturnSuccess);
    return UNITY_END();
}
