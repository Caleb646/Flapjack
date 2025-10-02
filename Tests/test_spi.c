#include "peripheral/bus/spi.h"
#include "unity/unity.h"
#include <stdbool.h>
#include <stdint.h>

#ifndef UNIT_TEST
#error "UNIT_TEST should be defined in this file"
#endif

// Test bus instance
static vSPIBus_t test_bus;

void setUp (void) {
    // Reset test bus for each test
    test_bus.isInitialized              = false;
    test_bus.busId                      = eSPI_1_BUS_ID;
    test_bus.deviceId                   = 1; // Use simple numeric ID
    test_bus.nDevices                   = 0;
    test_bus.activeTransaction.deviceId = 0;
    test_bus.activeTransaction.pNss     = NULL;
}

void tearDown (void) {
    // Clean up after each test
}

// Test SPIInit with invalid bus ID
void test_SPIInit_InvalidBusId_ReturnsFailure (void) {

    SPIInitConf_t conf            = { 0 };
    conf.deviceBoardConf.deviceId = eIMU_DEVICE_ID;
    conf.busBoardConf.busId       = eI2C_1_BUS_ID; // Wrong bus type

    eSTATUS_t status = SPIInit (conf, &test_bus);
    TEST_ASSERT_EQUAL (eSTATUS_FAILURE, status);
}

// Test bus ID validation
void test_BusIdValidation_SPIBusIds (void) {
    // Test that SPI bus IDs are valid
    TEST_ASSERT_TRUE (BUS_ID_IS_SPI (eSPI_1_BUS_ID));
    TEST_ASSERT_TRUE (BUS_ID_IS_SPI (eSPI_2_BUS_ID));
    TEST_ASSERT_TRUE (BUS_ID_IS_SPI (eSPI_3_BUS_ID));

    // Test that non-SPI bus IDs are rejected
    TEST_ASSERT_FALSE (BUS_ID_IS_SPI (eI2C_1_BUS_ID));
    TEST_ASSERT_FALSE (BUS_ID_IS_SPI (eUART_1_BUS_ID));
}

// Test SPI transaction state management
void test_SPITransaction_StateTracking (void) {
    // Setup a minimal bus state for testing transaction tracking
    test_bus.nDevices                = 1;
    test_bus.connectedDevices.ids[0] = eIMU_DEVICE_ID;

    // Initially no active transaction
    TEST_ASSERT_EQUAL (0, test_bus.activeTransaction.deviceId);
    TEST_ASSERT_NULL (test_bus.activeTransaction.pNss);

    // The transaction functions should handle null GPIO gracefully
    // This tests the basic transaction state tracking without GPIO dependencies
}

// Test basic data transfer function signatures
void test_SPIDataTransfer_FunctionSignatures (void) {
    // Test that the SPI data transfer functions exist and can be called
    // without crashing, even if they fail due to uninitialized bus

    uint8_t testData[4] = { 0x01, 0x02, 0x03, 0x04 };
    uint8_t rxData[4];

    // These should return failure but not crash
    eSTATUS_t readStatus =
    SPIRead_Blocking (&test_bus, eIMU_DEVICE_ID, rxData, sizeof (rxData));
    TEST_ASSERT_EQUAL (eSTATUS_FAILURE, readStatus);

    eSTATUS_t writeStatus =
    SPIWrite_Blocking (&test_bus, eIMU_DEVICE_ID, testData, sizeof (testData));
    TEST_ASSERT_EQUAL (eSTATUS_FAILURE, writeStatus);

    eSTATUS_t writeReadStatus =
    SPIWriteRead_Blocking (&test_bus, eIMU_DEVICE_ID, testData, rxData, sizeof (testData));
    TEST_ASSERT_EQUAL (eSTATUS_FAILURE, writeReadStatus);
}


// Run all tests
void test_spi_run_tests (void) {
    RUN_TEST (test_SPIInit_InvalidBusId_ReturnsFailure);
    RUN_TEST (test_BusIdValidation_SPIBusIds);
    RUN_TEST (test_SPITransaction_StateTracking);
    RUN_TEST (test_SPIDataTransfer_FunctionSignatures);
}

int main (void) {
    UNITY_BEGIN ();
    test_spi_run_tests ();
    return UNITY_END ();
}