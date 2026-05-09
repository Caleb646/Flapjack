#include "drivers/serial/uart.h"
#include "unity/unity.h"
#include <stdbool.h>
#include <stdint.h>

#ifndef UNIT_TEST
#error "UNIT_TEST should be defined in this file"
#endif

// Test bus instance
static vUARTBus_t test_bus;

void setUp (void) {
    // Reset test bus for each test
    test_bus.isInitialized = false;
    test_bus.busId         = eUART_1_BUS_ID;
    test_bus.deviceId      = 1; // Use simple numeric ID
    test_bus.rxCallback    = NULL;
    test_bus.txCallback    = NULL;
    test_bus.errorCallback = NULL;
}

void tearDown (void) {
    // Clean up after each test
}

// Test UARTGetBusById function
void test_UARTGetBusById_ValidId_ReturnsCorrectBus (void) {
    vUARTBus_t* pBus = UARTGetBusById (eUART_1_BUS_ID);
    TEST_ASSERT_NOT_NULL (pBus);
}

void test_UARTGetBusById_InvalidId_ReturnsNull (void) {
    vUARTBus_t* pBus = UARTGetBusById (eSPI_1_BUS_ID); // Wrong bus type
    TEST_ASSERT_NULL (pBus);
}

// Test UARTInit with invalid bus ID
void test_UARTInit_InvalidBusId_ReturnsFailure (void) {
    UARTInitConf_t conf           = { 0 };
    conf.deviceBoardConf.deviceId = eIMU_DEVICE_ID;
    conf.busBoardConf.busId       = eSPI_1_BUS_ID; // Wrong bus type

    eSTATUS_t status = UARTInit (conf, &test_bus);
    TEST_ASSERT_EQUAL (eSTATUS_FAILURE, status);
}

// Test UARTInit with zero baud rate
void test_UARTInit_ZeroBaudRate_ReturnsFailure (void) {
    UARTInitConf_t conf                      = { 0 };
    conf.deviceBoardConf.deviceId            = eIMU_DEVICE_ID;
    conf.busBoardConf.busId                  = eUART_1_BUS_ID;
    conf.busBoardConf.UARTBoardConf.baudRate = 0; // Invalid baud rate

    eSTATUS_t status = UARTInit (conf, &test_bus);
    TEST_ASSERT_EQUAL (eSTATUS_FAILURE, status);
}

// Test UARTInit with valid baud rate but missing GPIO
void test_UARTInit_ValidBaudRate_MissingGPIO_ReturnsFailure (void) {
    UARTInitConf_t conf                          = { 0 };
    conf.deviceBoardConf.deviceId                = eIMU_DEVICE_ID;
    conf.busBoardConf.busId                      = eUART_1_BUS_ID;
    conf.busBoardConf.UARTBoardConf.baudRate     = eUART_BAUD_115200;
    conf.busBoardConf.UARTBoardConf.pTxBoardConf = NULL; // Missing TX pin
    conf.busBoardConf.UARTBoardConf.pRxBoardConf = NULL; // Missing RX pin

    eSTATUS_t status = UARTInit (conf, &test_bus);
    TEST_ASSERT_EQUAL (eSTATUS_FAILURE, status);
}

// Test bus ID validation
void test_BusIdValidation_UARTBusIds (void) {
    // Test that UART bus IDs are valid
    TEST_ASSERT_TRUE (BUS_ID_IS_UART (eUART_1_BUS_ID));
    TEST_ASSERT_TRUE (BUS_ID_IS_UART (eUART_2_BUS_ID));
    TEST_ASSERT_TRUE (BUS_ID_IS_UART (eUART_3_BUS_ID));

    // Test that non-UART bus IDs are rejected
    TEST_ASSERT_FALSE (BUS_ID_IS_UART (eSPI_1_BUS_ID));
    TEST_ASSERT_FALSE (BUS_ID_IS_UART (eI2C_1_BUS_ID));
}

// Test UART baud rate constants
void test_UARTBaudRateConstants (void) {
    TEST_ASSERT_EQUAL (9600, eUART_BAUD_9600);
    TEST_ASSERT_EQUAL (19200, eUART_BAUD_19200);
    TEST_ASSERT_EQUAL (38400, eUART_BAUD_38400);
    TEST_ASSERT_EQUAL (57600, eUART_BAUD_57600);
    TEST_ASSERT_EQUAL (115200, eUART_BAUD_115200);
    TEST_ASSERT_EQUAL (230400, eUART_BAUD_230400);
}

void dummy_callback (eBUS_ID_t busId) {
    (void)busId;
}

// Test UART callback registration
void test_UARTRegisterCallback_NullBus_ReturnsFailure (void) {


    eSTATUS_t status = UARTRegisterCallback (NULL, eUART_CALLBACK_ID_RX, dummy_callback);
    TEST_ASSERT_EQUAL (eSTATUS_FAILURE, status);
}

void test_UARTRegisterCallback_NullCallback_ReturnsFailure (void) {
    // Mark bus as initialized for validation
    test_bus.isInitialized = true;

    eSTATUS_t status = UARTRegisterCallback (&test_bus, eUART_CALLBACK_ID_RX, NULL);
    TEST_ASSERT_EQUAL (eSTATUS_FAILURE, status);
}

void test_UARTRegisterCallback_InvalidCallbackId_ReturnsFailure (void) {
    // Mark bus as initialized for validation
    test_bus.isInitialized = true;

    eSTATUS_t status = UARTRegisterCallback (&test_bus, eUART_CALLBACK_ID_MAX, dummy_callback);
    TEST_ASSERT_EQUAL (eSTATUS_FAILURE, status);
}

void dummy_rx_callback (eBUS_ID_t busId) {
    (void)busId;
}
void dummy_tx_callback (eBUS_ID_t busId) {
    (void)busId;
}
void dummy_error_callback (eBUS_ID_t busId) {
    (void)busId;
}

void test_UARTRegisterCallback_ValidParameters_ReturnsSuccess (void) {

    // Mark bus as initialized for validation
    test_bus.isInitialized = true;

    eSTATUS_t status;

    status = UARTRegisterCallback (&test_bus, eUART_CALLBACK_ID_RX, dummy_rx_callback);
    TEST_ASSERT_EQUAL (eSTATUS_SUCCESS, status);
    TEST_ASSERT_EQUAL (dummy_rx_callback, test_bus.rxCallback);

    status = UARTRegisterCallback (&test_bus, eUART_CALLBACK_ID_TX, dummy_tx_callback);
    TEST_ASSERT_EQUAL (eSTATUS_SUCCESS, status);
    TEST_ASSERT_EQUAL (dummy_tx_callback, test_bus.txCallback);

    status = UARTRegisterCallback (&test_bus, eUART_CALLBACK_ID_ERROR, dummy_error_callback);
    TEST_ASSERT_EQUAL (eSTATUS_SUCCESS, status);
    TEST_ASSERT_EQUAL (dummy_error_callback, test_bus.errorCallback);
}

// Test UART interrupt enable
void test_UARTEnableInterrupts_UninitializedBus_ReturnsFailure (void) {
    test_bus.isInitialized = false;

    eSTATUS_t status = UARTEnableInterrupts (&test_bus, 0);
    TEST_ASSERT_EQUAL (eSTATUS_FAILURE, status);
}

void test_UARTEnableInterrupts_NullBus_ReturnsFailure (void) {
    eSTATUS_t status = UARTEnableInterrupts (NULL, 0);
    TEST_ASSERT_EQUAL (eSTATUS_FAILURE, status);
}

// Test basic data transfer function signatures
void test_UARTDataTransfer_FunctionSignatures (void) {
    // Test that the UART data transfer functions exist and can be called
    // without crashing, even if they fail due to uninitialized bus

    uint8_t testData[4] = { 0x01, 0x02, 0x03, 0x04 };
    uint8_t rxData[4];

    // These should return failure but not crash due to uninitialized bus
    eSTATUS_t readStatus = UARTRead_Blocking (&test_bus, eIMU_DEVICE_ID, rxData, sizeof (rxData));
    TEST_ASSERT_EQUAL (eSTATUS_FAILURE, readStatus);

    eSTATUS_t writeStatus = UARTWrite_Blocking (&test_bus, eIMU_DEVICE_ID, testData, sizeof (testData));
    TEST_ASSERT_EQUAL (eSTATUS_FAILURE, writeStatus);

    eSTATUS_t readITStatus = UARTRead_IT (&test_bus, eIMU_DEVICE_ID, rxData, sizeof (rxData));
    TEST_ASSERT_EQUAL (eSTATUS_FAILURE, readITStatus);
}

// Test UART data transfer with null parameters
void test_UARTDataTransfer_NullParameters_ReturnsFailure (void) {
    uint8_t testData[4] = { 0x01, 0x02, 0x03, 0x04 };

    // Mark bus as initialized for validation
    test_bus.isInitialized = true;

    // Test null data pointer
    eSTATUS_t status = UARTRead_Blocking (&test_bus, eIMU_DEVICE_ID, NULL, 4);
    TEST_ASSERT_EQUAL (eSTATUS_FAILURE, status);

    status = UARTWrite_Blocking (&test_bus, eIMU_DEVICE_ID, NULL, 4);
    TEST_ASSERT_EQUAL (eSTATUS_FAILURE, status);

    status = UARTRead_IT (&test_bus, eIMU_DEVICE_ID, NULL, 4);
    TEST_ASSERT_EQUAL (eSTATUS_FAILURE, status);

    // Test zero size
    status = UARTRead_Blocking (&test_bus, eIMU_DEVICE_ID, testData, 0);
    TEST_ASSERT_EQUAL (eSTATUS_FAILURE, status);

    status = UARTWrite_Blocking (&test_bus, eIMU_DEVICE_ID, testData, 0);
    TEST_ASSERT_EQUAL (eSTATUS_FAILURE, status);

    status = UARTRead_IT (&test_bus, eIMU_DEVICE_ID, testData, 0);
    TEST_ASSERT_EQUAL (eSTATUS_FAILURE, status);
}

// Test wrapper function signatures
void test_UARTWrapperFunctions_Signatures (void) {
    uint8_t testData[4] = { 0x01, 0x02, 0x03, 0x04 };
    uint8_t rxData[4];

    // Test wrapper functions exist and handle invalid context gracefully
    eSTATUS_t readStatus = UART_READ_BLOCKING (&test_bus, eIMU_DEVICE_ID, rxData, sizeof (rxData));
    TEST_ASSERT_EQUAL (eSTATUS_FAILURE, readStatus);

    eSTATUS_t writeStatus = UART_WRITE_BLOCKING (&test_bus, eIMU_DEVICE_ID, testData, sizeof (testData));
    TEST_ASSERT_EQUAL (eSTATUS_FAILURE, writeStatus);

    eSTATUS_t readITStatus = UART_READ_IT (&test_bus, eIMU_DEVICE_ID, rxData, sizeof (rxData));
    TEST_ASSERT_EQUAL (eSTATUS_FAILURE, readITStatus);
}

// Run all tests
void test_uart_run_tests (void) {
    RUN_TEST (test_UARTGetBusById_ValidId_ReturnsCorrectBus);
    RUN_TEST (test_UARTGetBusById_InvalidId_ReturnsNull);
    RUN_TEST (test_UARTInit_InvalidBusId_ReturnsFailure);
    RUN_TEST (test_UARTInit_ZeroBaudRate_ReturnsFailure);
    RUN_TEST (test_UARTInit_ValidBaudRate_MissingGPIO_ReturnsFailure);
    RUN_TEST (test_BusIdValidation_UARTBusIds);
    RUN_TEST (test_UARTBaudRateConstants);
    RUN_TEST (test_UARTRegisterCallback_NullBus_ReturnsFailure);
    RUN_TEST (test_UARTRegisterCallback_NullCallback_ReturnsFailure);
    RUN_TEST (test_UARTRegisterCallback_InvalidCallbackId_ReturnsFailure);
    RUN_TEST (test_UARTRegisterCallback_ValidParameters_ReturnsSuccess);
    RUN_TEST (test_UARTEnableInterrupts_UninitializedBus_ReturnsFailure);
    RUN_TEST (test_UARTEnableInterrupts_NullBus_ReturnsFailure);
    RUN_TEST (test_UARTDataTransfer_FunctionSignatures);
    RUN_TEST (test_UARTDataTransfer_NullParameters_ReturnsFailure);
    RUN_TEST (test_UARTWrapperFunctions_Signatures);
}

int main (void) {
    UNITY_BEGIN ();
    test_uart_run_tests ();
    return UNITY_END ();
}