#include "mem/queue.h"
#include "unity/unity.h"
#include <stdint.h>
#include <string.h>


#ifndef UNIT_TEST
#error "UNIT_TEST should be defined in this file"
#endif

// Test data structures
typedef struct {
    uint32_t id;
    uint16_t value;
    uint8_t flag;
} TestStruct;

// Test buffer and queue instances
static uint8_t test_buffer_uint8[8];
static Queue test_queue_uint8;

static uint32_t test_buffer_uint32[4];
static Queue test_queue_uint32;

static TestStruct test_buffer_struct[2];
static Queue test_queue_struct;

// Test: Queue initialization with valid parameters
void test_QueueInit_ValidParameters (void) {
    STATUS_TYPE result =
    QueueInit (&test_queue_uint8, test_buffer_uint8, 8, sizeof (uint8_t));

    TEST_ASSERT_EQUAL (eSTATUS_SUCCESS, result);
    TEST_ASSERT_EQUAL (8, QueueCapacity (&test_queue_uint8));
    TEST_ASSERT_EQUAL (0, QueueCount (&test_queue_uint8));
    TEST_ASSERT_TRUE (QueueIsEmpty (&test_queue_uint8));
    TEST_ASSERT_FALSE (QueueIsFull (&test_queue_uint8));
}

// Test: Queue initialization with NULL queue pointer
void test_QueueInit_NullQueue (void) {
    STATUS_TYPE result = QueueInit (NULL, test_buffer_uint8, 8, sizeof (uint8_t));
    TEST_ASSERT_EQUAL (eSTATUS_FAILURE, result);
}

// Test: Queue initialization with NULL buffer pointer
void test_QueueInit_NullBuffer (void) {
    STATUS_TYPE result = QueueInit (&test_queue_uint8, NULL, 8, sizeof (uint8_t));
    TEST_ASSERT_EQUAL (eSTATUS_FAILURE, result);
}

// Test: Queue initialization with zero capacity
void test_QueueInit_ZeroCapacity (void) {
    STATUS_TYPE result =
    QueueInit (&test_queue_uint8, test_buffer_uint8, 0, sizeof (uint8_t));
    TEST_ASSERT_EQUAL (eSTATUS_FAILURE, result);
}

// Test: Queue initialization with non-power-of-2 capacity
void test_QueueInit_NonPowerOf2Capacity (void) {
    // Test with capacity 3 (not a power of 2)
    STATUS_TYPE result =
    QueueInit (&test_queue_uint8, test_buffer_uint8, 3, sizeof (uint8_t));
    TEST_ASSERT_EQUAL (eSTATUS_FAILURE, result);

    // Test with capacity 5 (not a power of 2)
    result = QueueInit (&test_queue_uint8, test_buffer_uint8, 5, sizeof (uint8_t));
    TEST_ASSERT_EQUAL (eSTATUS_FAILURE, result);

    // Test with capacity 6 (not a power of 2)
    result = QueueInit (&test_queue_uint8, test_buffer_uint8, 6, sizeof (uint8_t));
    TEST_ASSERT_EQUAL (eSTATUS_FAILURE, result);

    // Test with capacity 7 (not a power of 2)
    result = QueueInit (&test_queue_uint8, test_buffer_uint8, 7, sizeof (uint8_t));
    TEST_ASSERT_EQUAL (eSTATUS_FAILURE, result);

    // Test with capacity 9 (not a power of 2)
    result = QueueInit (&test_queue_uint8, test_buffer_uint8, 9, sizeof (uint8_t));
    TEST_ASSERT_EQUAL (eSTATUS_FAILURE, result);
}

// Test: Queue initialization with zero element size
void test_QueueInit_ZeroElementSize (void) {
    STATUS_TYPE result = QueueInit (&test_queue_uint8, test_buffer_uint8, 8, 0);
    TEST_ASSERT_EQUAL (eSTATUS_FAILURE, result);
}

// Test: Basic enqueue and dequeue operations
void test_QueueEnqueueDequeue_Basic (void) {
    QueueInit (&test_queue_uint8, test_buffer_uint8, 8, sizeof (uint8_t));

    uint8_t input_value  = 42;
    uint8_t output_value = 0;

    // Enqueue
    STATUS_TYPE result = QueueEnqueue (&test_queue_uint8, &input_value);
    TEST_ASSERT_EQUAL (eSTATUS_SUCCESS, result);
    TEST_ASSERT_EQUAL (1, QueueCount (&test_queue_uint8));
    TEST_ASSERT_FALSE (QueueIsEmpty (&test_queue_uint8));
    TEST_ASSERT_FALSE (QueueIsFull (&test_queue_uint8));

    // Dequeue
    result = QueueDequeue (&test_queue_uint8, &output_value);
    TEST_ASSERT_EQUAL (eSTATUS_SUCCESS, result);
    TEST_ASSERT_EQUAL (input_value, output_value);
    TEST_ASSERT_EQUAL (0, QueueCount (&test_queue_uint8));
    TEST_ASSERT_TRUE (QueueIsEmpty (&test_queue_uint8));
    TEST_ASSERT_FALSE (QueueIsFull (&test_queue_uint8));
}

// Test: Enqueue with NULL queue pointer
void test_QueueEnqueue_NullQueue (void) {
    uint8_t value      = 42;
    STATUS_TYPE result = QueueEnqueue (NULL, &value);
    TEST_ASSERT_EQUAL (eSTATUS_FAILURE, result);
}

// Test: Enqueue with NULL element pointer
void test_QueueEnqueue_NullElement (void) {
    QueueInit (&test_queue_uint8, test_buffer_uint8, 8, sizeof (uint8_t));
    STATUS_TYPE result = QueueEnqueue (&test_queue_uint8, NULL);
    TEST_ASSERT_EQUAL (eSTATUS_FAILURE, result);
}

// Test: Dequeue with NULL queue pointer
void test_QueueDequeue_NullQueue (void) {
    uint8_t value;
    STATUS_TYPE result = QueueDequeue (NULL, &value);
    TEST_ASSERT_EQUAL (eSTATUS_FAILURE, result);
}

// Test: Dequeue with NULL element pointer
void test_QueueDequeue_NullElement (void) {
    QueueInit (&test_queue_uint8, test_buffer_uint8, 8, sizeof (uint8_t));
    STATUS_TYPE result = QueueDequeue (&test_queue_uint8, NULL);
    TEST_ASSERT_EQUAL (eSTATUS_FAILURE, result);
}

// Test: Dequeue from empty queue
void test_QueueDequeue_EmptyQueue (void) {
    QueueInit (&test_queue_uint8, test_buffer_uint8, 8, sizeof (uint8_t));

    uint8_t value;
    STATUS_TYPE result = QueueDequeue (&test_queue_uint8, &value);
    TEST_ASSERT_EQUAL (eSTATUS_FAILURE, result);
}

// Test: Fill queue to capacity
void test_QueueFillToCapacity (void) {
    QueueInit (&test_queue_uint8, test_buffer_uint8, 8, sizeof (uint8_t));

    // Fill the queue
    for (uint8_t i = 0; i < 8; i++) {
        STATUS_TYPE result = QueueEnqueue (&test_queue_uint8, &i);
        TEST_ASSERT_EQUAL (eSTATUS_SUCCESS, result);
        TEST_ASSERT_EQUAL (i + 1, QueueCount (&test_queue_uint8));
    }

    TEST_ASSERT_TRUE (QueueIsFull (&test_queue_uint8));
    TEST_ASSERT_FALSE (QueueIsEmpty (&test_queue_uint8));
    TEST_ASSERT_EQUAL (8, QueueCount (&test_queue_uint8));
}

// Test: Enqueue to full queue
void test_QueueEnqueue_FullQueue (void) {
    QueueInit (&test_queue_uint8, test_buffer_uint8, 8, sizeof (uint8_t));

    // Fill the queue
    for (uint8_t i = 0; i < 8; i++) {
        QueueEnqueue (&test_queue_uint8, &i);
    }

    // Try to enqueue one more
    uint8_t extra_value = 99;
    STATUS_TYPE result  = QueueEnqueue (&test_queue_uint8, &extra_value);
    TEST_ASSERT_EQUAL (eSTATUS_FAILURE, result);
    TEST_ASSERT_EQUAL (8, QueueCount (&test_queue_uint8));
}

// Test: FIFO ordering
void test_QueueFIFOOrdering (void) {
    QueueInit (&test_queue_uint8, test_buffer_uint8, 8, sizeof (uint8_t));

    // Enqueue values 0-7
    for (uint8_t i = 0; i < 8; i++) {
        QueueEnqueue (&test_queue_uint8, &i);
    }

    // Dequeue and verify FIFO order
    for (uint8_t i = 0; i < 8; i++) {
        uint8_t value;
        STATUS_TYPE result = QueueDequeue (&test_queue_uint8, &value);
        TEST_ASSERT_EQUAL (eSTATUS_SUCCESS, result);
        TEST_ASSERT_EQUAL (i, value);
    }

    TEST_ASSERT_TRUE (QueueIsEmpty (&test_queue_uint8));
}

// Test: Circular buffer behavior
void test_QueueCircularBehavior (void) {
    QueueInit (&test_queue_uint8, test_buffer_uint8, 4, sizeof (uint8_t));

    // Fill queue
    for (uint8_t i = 0; i < 4; i++) {
        QueueEnqueue (&test_queue_uint8, &i);
    }

    // Remove some elements
    uint8_t value;
    QueueDequeue (&test_queue_uint8, &value);
    QueueDequeue (&test_queue_uint8, &value);

    // Add more elements (should wrap around)
    uint8_t new_val1 = 10;
    uint8_t new_val2 = 11;
    TEST_ASSERT_EQUAL (eSTATUS_SUCCESS, QueueEnqueue (&test_queue_uint8, &new_val1));
    TEST_ASSERT_EQUAL (eSTATUS_SUCCESS, QueueEnqueue (&test_queue_uint8, &new_val2));

    // Verify correct order
    QueueDequeue (&test_queue_uint8, &value);
    TEST_ASSERT_EQUAL (2, value);
    QueueDequeue (&test_queue_uint8, &value);
    TEST_ASSERT_EQUAL (3, value);
    QueueDequeue (&test_queue_uint8, &value);
    TEST_ASSERT_EQUAL (10, value);
    QueueDequeue (&test_queue_uint8, &value);
    TEST_ASSERT_EQUAL (11, value);

    TEST_ASSERT_TRUE (QueueIsEmpty (&test_queue_uint8));
}

// Test: Peek functionality
void test_QueuePeek_Basic (void) {
    QueueInit (&test_queue_uint8, test_buffer_uint8, 8, sizeof (uint8_t));

    uint8_t input_value = 42;
    uint8_t peek_value  = 0;

    // Enqueue a value
    QueueEnqueue (&test_queue_uint8, &input_value);

    // Peek should return the value without removing it
    STATUS_TYPE result = QueuePeek (&test_queue_uint8, &peek_value);
    TEST_ASSERT_EQUAL (eSTATUS_SUCCESS, result);
    TEST_ASSERT_EQUAL (input_value, peek_value);
    TEST_ASSERT_EQUAL (1, QueueCount (&test_queue_uint8));

    // Peek again should return the same value
    peek_value = 0;
    result     = QueuePeek (&test_queue_uint8, &peek_value);
    TEST_ASSERT_EQUAL (eSTATUS_SUCCESS, result);
    TEST_ASSERT_EQUAL (input_value, peek_value);
    TEST_ASSERT_EQUAL (1, QueueCount (&test_queue_uint8));
}

// Test: Peek from empty queue
void test_QueuePeek_EmptyQueue (void) {
    QueueInit (&test_queue_uint8, test_buffer_uint8, 8, sizeof (uint8_t));

    uint8_t value;
    STATUS_TYPE result = QueuePeek (&test_queue_uint8, &value);
    TEST_ASSERT_EQUAL (eSTATUS_FAILURE, result);
}

// Test: Peek with NULL parameters
void test_QueuePeek_NullParameters (void) {
    QueueInit (&test_queue_uint8, test_buffer_uint8, 8, sizeof (uint8_t));

    uint8_t value = 42;
    QueueEnqueue (&test_queue_uint8, &value);

    // NULL queue
    STATUS_TYPE result = QueuePeek (NULL, &value);
    TEST_ASSERT_EQUAL (eSTATUS_FAILURE, result);

    // NULL output element
    result = QueuePeek (&test_queue_uint8, NULL);
    TEST_ASSERT_EQUAL (eSTATUS_FAILURE, result);
}

// Test: Clear queue
void test_QueueClear (void) {
    QueueInit (&test_queue_uint8, test_buffer_uint8, 8, sizeof (uint8_t));

    // Add some elements
    for (uint8_t i = 0; i < 5; i++) {
        QueueEnqueue (&test_queue_uint8, &i);
    }

    TEST_ASSERT_EQUAL (5, QueueCount (&test_queue_uint8));
    TEST_ASSERT_FALSE (QueueIsEmpty (&test_queue_uint8));

    // Clear the queue
    QueueClear (&test_queue_uint8);

    TEST_ASSERT_EQUAL (0, QueueCount (&test_queue_uint8));
    TEST_ASSERT_TRUE (QueueIsEmpty (&test_queue_uint8));
    TEST_ASSERT_FALSE (QueueIsFull (&test_queue_uint8));

    // Should be able to add elements again
    uint8_t value      = 99;
    STATUS_TYPE result = QueueEnqueue (&test_queue_uint8, &value);
    TEST_ASSERT_EQUAL (eSTATUS_SUCCESS, result);
}

// Test: Clear with NULL queue
void test_QueueClear_NullQueue (void) {
    // Should not crash
    QueueClear (NULL);
}

// Test: Different data types - uint32_t
void test_QueueUint32 (void) {
    QueueInit (&test_queue_uint32, test_buffer_uint32, 4, sizeof (uint32_t));

    uint32_t input_values[] = { 0x12345678, 0xABCDEF00, 0xDEADBEEF, 0xCAFEBABE };
    uint32_t output_value;

    // Enqueue all values
    for (int i = 0; i < 4; i++) {
        STATUS_TYPE result = QueueEnqueue (&test_queue_uint32, &input_values[i]);
        TEST_ASSERT_EQUAL (eSTATUS_SUCCESS, result);
    }

    // Dequeue and verify
    for (int i = 0; i < 4; i++) {
        STATUS_TYPE result = QueueDequeue (&test_queue_uint32, &output_value);
        TEST_ASSERT_EQUAL (eSTATUS_SUCCESS, result);
        TEST_ASSERT_EQUAL (input_values[i], output_value);
    }
}

// Test: Different data types - struct
void test_QueueStruct (void) {
    QueueInit (&test_queue_struct, test_buffer_struct, 2, sizeof (TestStruct));

    TestStruct input_structs[] = { { .id = 1, .value = 100, .flag = 0xAA },
                                   { .id = 2, .value = 200, .flag = 0xBB } };
    TestStruct output_struct;

    // Enqueue all structs
    for (int i = 0; i < 2; i++) {
        STATUS_TYPE result = QueueEnqueue (&test_queue_struct, &input_structs[i]);
        TEST_ASSERT_EQUAL (eSTATUS_SUCCESS, result);
    }

    // Dequeue and verify
    for (int i = 0; i < 2; i++) {
        STATUS_TYPE result = QueueDequeue (&test_queue_struct, &output_struct);
        TEST_ASSERT_EQUAL (eSTATUS_SUCCESS, result);
        TEST_ASSERT_EQUAL (input_structs[i].id, output_struct.id);
        TEST_ASSERT_EQUAL (input_structs[i].value, output_struct.value);
        TEST_ASSERT_EQUAL (input_structs[i].flag, output_struct.flag);
    }
}

// Test: Utility functions with NULL queue
void test_QueueUtilities_NullQueue (void) {
    TEST_ASSERT_TRUE (QueueIsEmpty (NULL));
    TEST_ASSERT_TRUE (QueueIsFull (NULL));
    TEST_ASSERT_EQUAL (0, QueueCount (NULL));
    TEST_ASSERT_EQUAL (0, QueueCapacity (NULL));
}

// Test: Multiple enqueue/dequeue cycles
void test_QueueMultipleCycles (void) {
    QueueInit (&test_queue_uint8, test_buffer_uint8, 4, sizeof (uint8_t));

    // Perform multiple cycles of fill/empty
    for (int cycle = 0; cycle < 3; cycle++) {
        // Fill queue
        for (uint8_t i = 0; i < 4; i++) {
            uint8_t value      = (cycle * 10) + i;
            STATUS_TYPE result = QueueEnqueue (&test_queue_uint8, &value);
            TEST_ASSERT_EQUAL (eSTATUS_SUCCESS, result);
        }

        TEST_ASSERT_TRUE (QueueIsFull (&test_queue_uint8));

        // Empty queue
        for (uint8_t i = 0; i < 4; i++) {
            uint8_t expected_value = (cycle * 10) + i;
            uint8_t actual_value;
            STATUS_TYPE result = QueueDequeue (&test_queue_uint8, &actual_value);
            TEST_ASSERT_EQUAL (eSTATUS_SUCCESS, result);
            TEST_ASSERT_EQUAL (expected_value, actual_value);
        }

        TEST_ASSERT_TRUE (QueueIsEmpty (&test_queue_uint8));
    }
}

// Test: Peek with multiple elements
void test_QueuePeekMultiple (void) {
    QueueInit (&test_queue_uint8, test_buffer_uint8, 8, sizeof (uint8_t));

    // Add multiple elements
    for (uint8_t i = 0; i < 5; i++) {
        QueueEnqueue (&test_queue_uint8, &i);
    }

    // Peek should always return the first element
    for (int i = 0; i < 3; i++) {
        uint8_t peek_value;
        STATUS_TYPE result = QueuePeek (&test_queue_uint8, &peek_value);
        TEST_ASSERT_EQUAL (eSTATUS_SUCCESS, result);
        TEST_ASSERT_EQUAL (0, peek_value);
        TEST_ASSERT_EQUAL (5, QueueCount (&test_queue_uint8));
    }

    // Dequeue one element
    uint8_t dequeue_value;
    QueueDequeue (&test_queue_uint8, &dequeue_value);
    TEST_ASSERT_EQUAL (0, dequeue_value);

    // Peek should now return the second element
    uint8_t peek_value;
    STATUS_TYPE result = QueuePeek (&test_queue_uint8, &peek_value);
    TEST_ASSERT_EQUAL (eSTATUS_SUCCESS, result);
    TEST_ASSERT_EQUAL (1, peek_value);
    TEST_ASSERT_EQUAL (4, QueueCount (&test_queue_uint8));
}
