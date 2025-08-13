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

void test_QueueInit_ValidParameters (void) {
    eSTATUS_t result =
    QueueInit (&test_queue_uint8, test_buffer_uint8, 8, sizeof (uint8_t), FALSE);

    TEST_ASSERT_EQUAL (eSTATUS_SUCCESS, result);
    TEST_ASSERT_EQUAL (8, QueueGetCapacity (&test_queue_uint8));
    TEST_ASSERT_EQUAL (0, QueueGetElementCount (&test_queue_uint8));
    TEST_ASSERT_TRUE (QueueIsEmpty (&test_queue_uint8));
    TEST_ASSERT_FALSE (QueueIsFull (&test_queue_uint8));
}

void test_QueueInit_NullQueue (void) {
    eSTATUS_t result =
    QueueInit (NULL, test_buffer_uint8, 8, sizeof (uint8_t), FALSE);
    TEST_ASSERT_EQUAL (eSTATUS_FAILURE, result);
}

void test_QueueInit_NullBuffer (void) {
    eSTATUS_t result =
    QueueInit (&test_queue_uint8, NULL, 8, sizeof (uint8_t), FALSE);
    TEST_ASSERT_EQUAL (eSTATUS_FAILURE, result);
}

void test_QueueInit_ZeroCapacity (void) {
    eSTATUS_t result =
    QueueInit (&test_queue_uint8, test_buffer_uint8, 0, sizeof (uint8_t), FALSE);
    TEST_ASSERT_EQUAL (eSTATUS_FAILURE, result);
}

void test_QueueInit_NonPowerOf2Capacity (void) {
    // Test with capacity 3 (not a power of 2)
    eSTATUS_t result =
    QueueInit (&test_queue_uint8, test_buffer_uint8, 3, sizeof (uint8_t), FALSE);
    TEST_ASSERT_EQUAL (eSTATUS_FAILURE, result);

    // Test with capacity 5 (not a power of 2)
    result =
    QueueInit (&test_queue_uint8, test_buffer_uint8, 5, sizeof (uint8_t), FALSE);
    TEST_ASSERT_EQUAL (eSTATUS_FAILURE, result);

    // Test with capacity 6 (not a power of 2)
    result =
    QueueInit (&test_queue_uint8, test_buffer_uint8, 6, sizeof (uint8_t), FALSE);
    TEST_ASSERT_EQUAL (eSTATUS_FAILURE, result);

    // Test with capacity 7 (not a power of 2)
    result =
    QueueInit (&test_queue_uint8, test_buffer_uint8, 7, sizeof (uint8_t), FALSE);
    TEST_ASSERT_EQUAL (eSTATUS_FAILURE, result);

    // Test with capacity 9 (not a power of 2)
    result =
    QueueInit (&test_queue_uint8, test_buffer_uint8, 9, sizeof (uint8_t), FALSE);
    TEST_ASSERT_EQUAL (eSTATUS_FAILURE, result);
}

void test_QueueInit_ZeroElementSize (void) {
    eSTATUS_t result = QueueInit (&test_queue_uint8, test_buffer_uint8, 8, 0, FALSE);
    TEST_ASSERT_EQUAL (eSTATUS_FAILURE, result);
}

void test_QueueEnqueueDequeue_Basic (void) {
    QueueInit (&test_queue_uint8, test_buffer_uint8, 8, sizeof (uint8_t), FALSE);

    uint8_t input_value  = 42;
    uint8_t output_value = 0;

    eSTATUS_t result = Queue_Push (&test_queue_uint8, &input_value);
    TEST_ASSERT_EQUAL (eSTATUS_SUCCESS, result);
    TEST_ASSERT_EQUAL (1, QueueGetElementCount (&test_queue_uint8));
    TEST_ASSERT_FALSE (QueueIsEmpty (&test_queue_uint8));
    TEST_ASSERT_FALSE (QueueIsFull (&test_queue_uint8));

    result = Queue_Pop (&test_queue_uint8, &output_value);
    TEST_ASSERT_EQUAL (eSTATUS_SUCCESS, result);
    TEST_ASSERT_EQUAL (input_value, output_value);
    TEST_ASSERT_EQUAL (0, QueueGetElementCount (&test_queue_uint8));
    TEST_ASSERT_TRUE (QueueIsEmpty (&test_queue_uint8));
    TEST_ASSERT_FALSE (QueueIsFull (&test_queue_uint8));
}

void test_QueueEnqueue_NullQueue (void) {
    uint8_t value    = 42;
    eSTATUS_t result = Queue_Push (NULL, &value);
    TEST_ASSERT_EQUAL (eSTATUS_FAILURE, result);
}

void test_QueueEnqueue_NullElement (void) {
    QueueInit (&test_queue_uint8, test_buffer_uint8, 8, sizeof (uint8_t), FALSE);
    eSTATUS_t result = Queue_Push (&test_queue_uint8, NULL);
    TEST_ASSERT_EQUAL (eSTATUS_FAILURE, result);
}

void test_QueueDequeue_NullQueue (void) {
    uint8_t value;
    eSTATUS_t result = Queue_Pop (NULL, &value);
    TEST_ASSERT_EQUAL (eSTATUS_FAILURE, result);
}

void test_QueueDequeue_NullElement (void) {
    QueueInit (&test_queue_uint8, test_buffer_uint8, 8, sizeof (uint8_t), FALSE);
    eSTATUS_t result = Queue_Pop (&test_queue_uint8, NULL);
    TEST_ASSERT_EQUAL (eSTATUS_FAILURE, result);
}

void test_QueueDequeue_EmptyQueue (void) {
    QueueInit (&test_queue_uint8, test_buffer_uint8, 8, sizeof (uint8_t), FALSE);

    uint8_t value;
    eSTATUS_t result = Queue_Pop (&test_queue_uint8, &value);
    TEST_ASSERT_EQUAL (eSTATUS_FAILURE, result);
}

void test_QueueFillToCapacity (void) {
    QueueInit (&test_queue_uint8, test_buffer_uint8, 8, sizeof (uint8_t), FALSE);

    for (uint8_t i = 0; i < 8; i++) {
        eSTATUS_t result = Queue_Push (&test_queue_uint8, &i);
        TEST_ASSERT_EQUAL (eSTATUS_SUCCESS, result);
        TEST_ASSERT_EQUAL (i + 1, QueueGetElementCount (&test_queue_uint8));
    }

    TEST_ASSERT_TRUE (QueueIsFull (&test_queue_uint8));
    TEST_ASSERT_FALSE (QueueIsEmpty (&test_queue_uint8));
    TEST_ASSERT_EQUAL (8, QueueGetElementCount (&test_queue_uint8));
}

void test_QueueEnqueue_FullQueue (void) {
    QueueInit (&test_queue_uint8, test_buffer_uint8, 8, sizeof (uint8_t), FALSE);

    for (uint8_t i = 0; i < 8; i++) {
        Queue_Push (&test_queue_uint8, &i);
    }

    uint8_t extra_value = 99;
    eSTATUS_t result    = Queue_Push (&test_queue_uint8, &extra_value);
    TEST_ASSERT_EQUAL (eSTATUS_FAILURE, result);
    TEST_ASSERT_EQUAL (8, QueueGetElementCount (&test_queue_uint8));
}

void test_QueueFIFOOrdering (void) {
    QueueInit (&test_queue_uint8, test_buffer_uint8, 8, sizeof (uint8_t), FALSE);

    for (uint8_t i = 0; i < 8; i++) {
        Queue_Push (&test_queue_uint8, &i);
    }

    for (uint8_t i = 0; i < 8; i++) {
        uint8_t value;
        eSTATUS_t result = Queue_Pop (&test_queue_uint8, &value);
        TEST_ASSERT_EQUAL (eSTATUS_SUCCESS, result);
        TEST_ASSERT_EQUAL (i, value);
    }

    TEST_ASSERT_TRUE (QueueIsEmpty (&test_queue_uint8));
}

void test_QueueCircularBehavior (void) {
    QueueInit (&test_queue_uint8, test_buffer_uint8, 4, sizeof (uint8_t), FALSE);

    for (uint8_t i = 0; i < 4; i++) {
        Queue_Push (&test_queue_uint8, &i);
    }

    uint8_t value;
    Queue_Pop (&test_queue_uint8, &value);
    Queue_Pop (&test_queue_uint8, &value);

    uint8_t new_val1 = 10;
    uint8_t new_val2 = 11;
    TEST_ASSERT_EQUAL (eSTATUS_SUCCESS, Queue_Push (&test_queue_uint8, &new_val1));
    TEST_ASSERT_EQUAL (eSTATUS_SUCCESS, Queue_Push (&test_queue_uint8, &new_val2));

    Queue_Pop (&test_queue_uint8, &value);
    TEST_ASSERT_EQUAL (2, value);
    Queue_Pop (&test_queue_uint8, &value);
    TEST_ASSERT_EQUAL (3, value);
    Queue_Pop (&test_queue_uint8, &value);
    TEST_ASSERT_EQUAL (10, value);
    Queue_Pop (&test_queue_uint8, &value);
    TEST_ASSERT_EQUAL (11, value);

    TEST_ASSERT_TRUE (QueueIsEmpty (&test_queue_uint8));
}

void test_QueuePeek_Basic (void) {
    QueueInit (&test_queue_uint8, test_buffer_uint8, 8, sizeof (uint8_t), FALSE);

    uint8_t input_value = 42;
    uint8_t peek_value  = 0;

    Queue_Push (&test_queue_uint8, &input_value);

    eSTATUS_t result = Queue_Peek (&test_queue_uint8, &peek_value);
    TEST_ASSERT_EQUAL (eSTATUS_SUCCESS, result);
    TEST_ASSERT_EQUAL (input_value, peek_value);
    TEST_ASSERT_EQUAL (1, QueueGetElementCount (&test_queue_uint8));

    peek_value = 0;
    result     = Queue_Peek (&test_queue_uint8, &peek_value);
    TEST_ASSERT_EQUAL (eSTATUS_SUCCESS, result);
    TEST_ASSERT_EQUAL (input_value, peek_value);
    TEST_ASSERT_EQUAL (1, QueueGetElementCount (&test_queue_uint8));
}

void test_QueuePeek_EmptyQueue (void) {
    QueueInit (&test_queue_uint8, test_buffer_uint8, 8, sizeof (uint8_t), FALSE);

    uint8_t value;
    eSTATUS_t result = Queue_Peek (&test_queue_uint8, &value);
    TEST_ASSERT_EQUAL (eSTATUS_FAILURE, result);
}

void test_QueuePeek_NullParameters (void) {
    QueueInit (&test_queue_uint8, test_buffer_uint8, 8, sizeof (uint8_t), FALSE);

    uint8_t value = 42;
    Queue_Push (&test_queue_uint8, &value);

    eSTATUS_t result = Queue_Peek (NULL, &value);
    TEST_ASSERT_EQUAL (eSTATUS_FAILURE, result);

    result = Queue_Peek (&test_queue_uint8, NULL);
    TEST_ASSERT_EQUAL (eSTATUS_FAILURE, result);
}

void test_QueueClear (void) {
    QueueInit (&test_queue_uint8, test_buffer_uint8, 8, sizeof (uint8_t), FALSE);

    for (uint8_t i = 0; i < 5; i++) {
        Queue_Push (&test_queue_uint8, &i);
    }

    TEST_ASSERT_EQUAL (5, QueueGetElementCount (&test_queue_uint8));
    TEST_ASSERT_FALSE (QueueIsEmpty (&test_queue_uint8));

    Queue_Clear (&test_queue_uint8);

    TEST_ASSERT_EQUAL (0, QueueGetElementCount (&test_queue_uint8));
    TEST_ASSERT_TRUE (QueueIsEmpty (&test_queue_uint8));
    TEST_ASSERT_FALSE (QueueIsFull (&test_queue_uint8));

    uint8_t value    = 99;
    eSTATUS_t result = Queue_Push (&test_queue_uint8, &value);
    TEST_ASSERT_EQUAL (eSTATUS_SUCCESS, result);
}

void test_QueueUint32 (void) {
    QueueInit (&test_queue_uint32, test_buffer_uint32, 4, sizeof (uint32_t), FALSE);

    uint32_t input_values[] = { 0x12345678, 0xABCDEF00, 0xDEADBEEF, 0xCAFEBABE };
    uint32_t output_value;

    for (int i = 0; i < 4; i++) {
        eSTATUS_t result = Queue_Push (&test_queue_uint32, &input_values[i]);
        TEST_ASSERT_EQUAL (eSTATUS_SUCCESS, result);
    }

    for (int i = 0; i < 4; i++) {
        eSTATUS_t result = Queue_Pop (&test_queue_uint32, &output_value);
        TEST_ASSERT_EQUAL (eSTATUS_SUCCESS, result);
        TEST_ASSERT_EQUAL (input_values[i], output_value);
    }
}

void test_QueueStruct (void) {
    QueueInit (&test_queue_struct, test_buffer_struct, 2, sizeof (TestStruct), FALSE);

    TestStruct input_structs[] = { { .id = 1, .value = 100, .flag = 0xAA },
                                   { .id = 2, .value = 200, .flag = 0xBB } };
    TestStruct output_struct;

    for (int i = 0; i < 2; i++) {
        eSTATUS_t result = Queue_Push (&test_queue_struct, &input_structs[i]);
        TEST_ASSERT_EQUAL (eSTATUS_SUCCESS, result);
    }

    for (int i = 0; i < 2; i++) {
        eSTATUS_t result = Queue_Pop (&test_queue_struct, &output_struct);
        TEST_ASSERT_EQUAL (eSTATUS_SUCCESS, result);
        TEST_ASSERT_EQUAL (input_structs[i].id, output_struct.id);
        TEST_ASSERT_EQUAL (input_structs[i].value, output_struct.value);
        TEST_ASSERT_EQUAL (input_structs[i].flag, output_struct.flag);
    }
}

void test_QueueUtilities_NullQueue (void) {
    TEST_ASSERT_TRUE (QueueIsEmpty (NULL));
    TEST_ASSERT_TRUE (QueueIsFull (NULL));
    TEST_ASSERT_EQUAL (0, QueueGetElementCount (NULL));
    TEST_ASSERT_EQUAL (0, QueueGetCapacity (NULL));
}

void test_QueueMultipleCycles (void) {
    QueueInit (&test_queue_uint8, test_buffer_uint8, 4, sizeof (uint8_t), FALSE);

    for (int cycle = 0; cycle < 3; cycle++) {
        for (uint8_t i = 0; i < 4; i++) {
            uint8_t value    = (cycle * 10) + i;
            eSTATUS_t result = Queue_Push (&test_queue_uint8, &value);
            TEST_ASSERT_EQUAL (eSTATUS_SUCCESS, result);
        }

        TEST_ASSERT_TRUE (QueueIsFull (&test_queue_uint8));

        for (uint8_t i = 0; i < 4; i++) {
            uint8_t expected_value = (cycle * 10) + i;
            uint8_t actual_value;
            eSTATUS_t result = Queue_Pop (&test_queue_uint8, &actual_value);
            TEST_ASSERT_EQUAL (eSTATUS_SUCCESS, result);
            TEST_ASSERT_EQUAL (expected_value, actual_value);
        }

        TEST_ASSERT_TRUE (QueueIsEmpty (&test_queue_uint8));
    }
}

void test_QueuePeekMultiple (void) {
    QueueInit (&test_queue_uint8, test_buffer_uint8, 8, sizeof (uint8_t), FALSE);

    for (uint8_t i = 0; i < 5; i++) {
        Queue_Push (&test_queue_uint8, &i);
    }

    for (int i = 0; i < 3; i++) {
        uint8_t peek_value;
        eSTATUS_t result = Queue_Peek (&test_queue_uint8, &peek_value);
        TEST_ASSERT_EQUAL (eSTATUS_SUCCESS, result);
        TEST_ASSERT_EQUAL (0, peek_value);
        TEST_ASSERT_EQUAL (5, QueueGetElementCount (&test_queue_uint8));
    }

    uint8_t dequeue_value;
    Queue_Pop (&test_queue_uint8, &dequeue_value);
    TEST_ASSERT_EQUAL (0, dequeue_value);

    uint8_t peek_value;
    eSTATUS_t result = Queue_Peek (&test_queue_uint8, &peek_value);
    TEST_ASSERT_EQUAL (eSTATUS_SUCCESS, result);
    TEST_ASSERT_EQUAL (1, peek_value);
    TEST_ASSERT_EQUAL (4, QueueGetElementCount (&test_queue_uint8));
}
