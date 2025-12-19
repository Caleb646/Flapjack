#include "conf/board.h"
#include "conf/ids.h"
#include "core/core.h"
#include "hal.h"
#include "peripheral/gpio.h"
#include "unity/unity.h"


#ifndef UNIT_TEST
#error "UNIT_TEST should be defined in this file"
#endif

#define GPIO_MAX_COUNT (GPIO_MAX_PORTS * eGPIO_PINID_MAX)

// Test that GPIO indexing works correctly for different port/pin combinations
void test_GPIO_array_indexing (void) {
    // Test various GPIO ID calculations and array access

    // Test Port A pins
    eGPIO_ID_t gpio_a0  = GPIO_ID_MAKE (eGPIO_PORTID_A, eGPIO_PINID_0);
    eGPIO_ID_t gpio_a1  = GPIO_ID_MAKE (eGPIO_PORTID_A, eGPIO_PINID_1);
    eGPIO_ID_t gpio_a15 = GPIO_ID_MAKE (eGPIO_PORTID_A, eGPIO_PINID_15);

    // Test Port B pins
    eGPIO_ID_t gpio_b0  = GPIO_ID_MAKE (eGPIO_PORTID_B, eGPIO_PINID_0);
    eGPIO_ID_t gpio_b15 = GPIO_ID_MAKE (eGPIO_PORTID_B, eGPIO_PINID_15);

    // Test Port C pins
    eGPIO_ID_t gpio_c0  = GPIO_ID_MAKE (eGPIO_PORTID_C, eGPIO_PINID_0);
    eGPIO_ID_t gpio_c15 = GPIO_ID_MAKE (eGPIO_PORTID_C, eGPIO_PINID_15);

    // Verify that we can get valid vIO structures for these GPIOs
    vIO_t* pIO_a0  = GPIOGetIOfromId (gpio_a0);
    vIO_t* pIO_a1  = GPIOGetIOfromId (gpio_a1);
    vIO_t* pIO_a15 = GPIOGetIOfromId (gpio_a15);
    vIO_t* pIO_b0  = GPIOGetIOfromId (gpio_b0);
    vIO_t* pIO_b15 = GPIOGetIOfromId (gpio_b15);
    vIO_t* pIO_c0  = GPIOGetIOfromId (gpio_c0);
    vIO_t* pIO_c15 = GPIOGetIOfromId (gpio_c15);

    // All should return valid pointers
    TEST_ASSERT_NOT_NULL (pIO_a0);
    TEST_ASSERT_NOT_NULL (pIO_a1);
    TEST_ASSERT_NOT_NULL (pIO_a15);
    TEST_ASSERT_NOT_NULL (pIO_b0);
    TEST_ASSERT_NOT_NULL (pIO_b15);
    TEST_ASSERT_NOT_NULL (pIO_c0);
    TEST_ASSERT_NOT_NULL (pIO_c15);

    // Verify that different GPIOs return different structures
    TEST_ASSERT_NOT_EQUAL (pIO_a0, pIO_a1);
    TEST_ASSERT_NOT_EQUAL (pIO_a0, pIO_b0);
    TEST_ASSERT_NOT_EQUAL (pIO_b0, pIO_c0);

    // Verify that accessing the same GPIO twice returns the same structure
    vIO_t* pIO_a0_again = GPIOGetIOfromId (gpio_a0);
    TEST_ASSERT_EQUAL (pIO_a0, pIO_a0_again);
}

// Test that GPIO indexing handles boundary cases correctly
void test_GPIO_array_indexing_boundaries (void) {
    // Test first GPIO in each port
    eGPIO_ID_t first_a = GPIO_ID_MAKE (eGPIO_PORTID_A, eGPIO_PINID_0);
    eGPIO_ID_t first_b = GPIO_ID_MAKE (eGPIO_PORTID_B, eGPIO_PINID_0);
    eGPIO_ID_t first_c = GPIO_ID_MAKE (eGPIO_PORTID_C, eGPIO_PINID_0);

    // Test last GPIO in each port
    eGPIO_ID_t last_a = GPIO_ID_MAKE (eGPIO_PORTID_A, eGPIO_PINID_15);
    eGPIO_ID_t last_b = GPIO_ID_MAKE (eGPIO_PORTID_B, eGPIO_PINID_15);
    eGPIO_ID_t last_c = GPIO_ID_MAKE (eGPIO_PORTID_C, eGPIO_PINID_15);

    // All should return valid pointers
    TEST_ASSERT_NOT_NULL (GPIOGetIOfromId (first_a));
    TEST_ASSERT_NOT_NULL (GPIOGetIOfromId (first_b));
    TEST_ASSERT_NOT_NULL (GPIOGetIOfromId (first_c));
    TEST_ASSERT_NOT_NULL (GPIOGetIOfromId (last_a));
    TEST_ASSERT_NOT_NULL (GPIOGetIOfromId (last_b));
    TEST_ASSERT_NOT_NULL (GPIOGetIOfromId (last_c));

    // Test invalid GPIO IDs (should return NULL)
    eGPIO_ID_t invalid_gpio = 0xFF; // Invalid GPIO ID
    TEST_ASSERT_NULL (GPIOGetIOfromId (invalid_gpio));
}

// Test that GPIO ownership is properly tracked
void test_GPIO_ownership_tracking (void) {
    eGPIO_ID_t test_gpio = GPIO_ID_MAKE (eGPIO_PORTID_A, eGPIO_PINID_5);
    eDEVICE_ID_t device1 = eIMU_DEVICE_ID;
    eDEVICE_ID_t device2 = eGPS_DEVICE_ID;

    vIO_t* pIO = GPIOGetIOfromId (test_gpio);
    TEST_ASSERT_NOT_NULL (pIO);

    // Initially, GPIO should be unowned
    TEST_ASSERT_EQUAL (eDEVICE_ID_NULL, pIO->ownerId);

    // Create a test GPIO configuration with minimal setup
    GPIODesc_t test_config = { .id   = test_gpio,
                               .conf = { .mode      = 0x1, // Output mode
                                         .pull      = 0x0, // No pull
                                         .speed     = 0x0, // Low speed
                                         .alternate = 0 } };

    // Initialize GPIO with device1 - should succeed
    eSTATUS_t result = GPIOInit (device1, test_config);
    TEST_ASSERT_EQUAL (eSTATUS_OK, result);
    TEST_ASSERT_EQUAL (device1, pIO->ownerId);

    // Try to initialize same GPIO with device2 - should fail (already owned)
    result = GPIOInit (device2, test_config);
    TEST_ASSERT_EQUAL (eSTATUS_FAIL, result);
    TEST_ASSERT_EQUAL (device1, pIO->ownerId); // Should still be owned by device1

    // Free the GPIO
    result = GPIOFreeById (test_gpio);
    TEST_ASSERT_EQUAL (eSTATUS_OK, result);
    TEST_ASSERT_EQUAL (eDEVICE_ID_NULL, pIO->ownerId);

    // Now device2 should be able to claim it
    result = GPIOInit (device2, test_config);
    TEST_ASSERT_EQUAL (eSTATUS_OK, result);
    TEST_ASSERT_EQUAL (device2, pIO->ownerId);
}

// Test multiple GPIO allocations and freeing
void test_GPIO_multiple_allocations (void) {
    eDEVICE_ID_t devices[] = { eIMU_DEVICE_ID, eGPS_DEVICE_ID, eBARO_DEVICE_ID, eMAG_DEVICE_ID, eRF_RECEIVER_DEVICE_ID };

    eGPIO_ID_t gpios[] = { GPIO_ID_MAKE (eGPIO_PORTID_A, eGPIO_PINID_0),
                           GPIO_ID_MAKE (eGPIO_PORTID_A, eGPIO_PINID_1),
                           GPIO_ID_MAKE (eGPIO_PORTID_B, eGPIO_PINID_0),
                           GPIO_ID_MAKE (eGPIO_PORTID_B, eGPIO_PINID_1),
                           GPIO_ID_MAKE (eGPIO_PORTID_C, eGPIO_PINID_0) };

    const uint8_t num_gpios = sizeof (gpios) / sizeof (gpios[0]);

    // Allocate all GPIOs
    for (uint8_t i = 0; i < num_gpios; i++) {
        GPIODesc_t config = { .id   = gpios[i],
                              .conf = { .mode      = 0x1, // Output mode
                                        .pull      = 0x0, // No pull
                                        .speed     = 0x0, // Low speed
                                        .alternate = 0 } };

        eSTATUS_t result = GPIOInit (devices[i], config);
        TEST_ASSERT_EQUAL (eSTATUS_OK, result);

        vIO_t* pIO = GPIOGetIOfromId (gpios[i]);
        TEST_ASSERT_EQUAL (devices[i], pIO->ownerId);
    }

    // Verify all are owned
    for (uint8_t i = 0; i < num_gpios; i++) {
        vIO_t* pIO = GPIOGetIOfromId (gpios[i]);
        TEST_ASSERT_EQUAL (devices[i], pIO->ownerId);
    }

    // Free odd-numbered GPIOs
    for (uint8_t i = 1; i < num_gpios; i += 2) {
        eSTATUS_t result = GPIOFreeById (gpios[i]);
        TEST_ASSERT_EQUAL (eSTATUS_OK, result);

        vIO_t* pIO = GPIOGetIOfromId (gpios[i]);
        TEST_ASSERT_EQUAL (eDEVICE_ID_NULL, pIO->ownerId);
    }

    // Verify even-numbered GPIOs are still owned
    for (uint8_t i = 0; i < num_gpios; i += 2) {
        vIO_t* pIO = GPIOGetIOfromId (gpios[i]);
        TEST_ASSERT_EQUAL (devices[i], pIO->ownerId);
    }

    // Free remaining GPIOs
    for (uint8_t i = 0; i < num_gpios; i += 2) {
        eSTATUS_t result = GPIOFreeById (gpios[i]);
        TEST_ASSERT_EQUAL (eSTATUS_OK, result);

        vIO_t* pIO = GPIOGetIOfromId (gpios[i]);
        TEST_ASSERT_EQUAL (eDEVICE_ID_NULL, pIO->ownerId);
    }
}

// Test error conditions
void test_GPIO_error_conditions (void) {
    // Test freeing invalid GPIO
    eGPIO_ID_t invalid_gpio = 0xFF;
    eSTATUS_t result        = GPIOFreeById (invalid_gpio);
    TEST_ASSERT_EQUAL (eSTATUS_FAIL, result);

    // Test freeing unowned GPIO (should succeed but have no effect)
    eGPIO_ID_t unowned_gpio = GPIO_ID_MAKE (eGPIO_PORTID_A, eGPIO_PINID_10);
    result                  = GPIOFreeById (unowned_gpio);
    TEST_ASSERT_EQUAL (eSTATUS_OK, result); // Should succeed even if not owned
}

// Test direct array access validation
void test_GPIO_array_direct_access (void) {
    // Get direct access to the GPIO array for testing
    IO_t* pIOs;
    uint32_t count;
    GPIOGetIOs (&pIOs, &count);

    TEST_ASSERT_NOT_NULL (pIOs);
    TEST_ASSERT_EQUAL (GPIO_MAX_COUNT, count);

    // Test a specific GPIO mapping
    eGPIO_ID_t test_gpio = GPIO_ID_MAKE (eGPIO_PORTID_A, eGPIO_PINID_5);
    vIO_t* pIO           = GPIOGetIOfromId (test_gpio);

    // Calculate expected array index
    uint8_t port_idx      = GPIO_ID_TO_PORT_IDX (test_gpio);
    uint8_t pin_idx       = GPIO_ID_TO_PIN_IDX (test_gpio);
    uint32_t expected_idx = port_idx * eGPIO_PINID_MAX + pin_idx;

    // Verify the returned pointer matches the expected array position
    TEST_ASSERT_EQUAL (&pIOs[expected_idx], pIO);
}

void setUp (void) {

    GPIOSystemInit ();
}

void tearDown (void) {
}

int main (void) {
    UNITY_BEGIN ();

    RUN_TEST (test_GPIO_array_indexing);
    RUN_TEST (test_GPIO_array_indexing_boundaries);
    RUN_TEST (test_GPIO_ownership_tracking);
    RUN_TEST (test_GPIO_multiple_allocations);
    RUN_TEST (test_GPIO_error_conditions);
    RUN_TEST (test_GPIO_array_direct_access);

    return UNITY_END ();
}