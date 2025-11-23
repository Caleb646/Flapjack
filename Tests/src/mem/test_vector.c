#include "core/stl/vector.h"
#include "unity/unity.h"
#include <stdbool.h>
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

// Test buffer and vector instances
static uint8_t test_buffer_uint8[8];
static Vector_t test_vector_uint8;

static uint32_t test_buffer_uint32[4];
static Vector_t test_vector_uint32;

static TestStruct test_buffer_struct[2];
static Vector_t test_vector_struct;

void setUp (void) {
    // Reset vectors for each test
    memset (&test_vector_uint8, 0, sizeof (Vector_t));
    memset (&test_vector_uint32, 0, sizeof (Vector_t));
    memset (&test_vector_struct, 0, sizeof (Vector_t));
    memset (test_buffer_uint8, 0, sizeof (test_buffer_uint8));
    memset (test_buffer_uint32, 0, sizeof (test_buffer_uint32));
    memset (test_buffer_struct, 0, sizeof (test_buffer_struct));
}

void tearDown (void) {
    // Clean up after each test
}

// Test Vector_Init function
void test_VectorInit_ValidParameters (void) {
    eSTATUS_t result = Vector_Init (&test_vector_uint8, test_buffer_uint8, 8, sizeof (uint8_t), false);

    TEST_ASSERT_EQUAL (eSTATUS_SUCCESS, result);
    TEST_ASSERT_EQUAL (8, Vector_Capacity (&test_vector_uint8));
    TEST_ASSERT_EQUAL (0, Vector_Size (&test_vector_uint8));
    TEST_ASSERT_TRUE (Vector_IsEmpty (&test_vector_uint8));
    TEST_ASSERT_FALSE (Vector_IsFull (&test_vector_uint8));
    TEST_ASSERT_EQUAL (0, test_vector_uint8.processID); // Non-shared
}

void test_VectorInit_SharedVector (void) {
    eSTATUS_t result = Vector_Init (&test_vector_uint8, test_buffer_uint8, 8, sizeof (uint8_t), true);

    TEST_ASSERT_EQUAL (eSTATUS_SUCCESS, result);
    TEST_ASSERT_EQUAL (1, test_vector_uint8.processID); // Shared
}

void test_VectorInit_NullVector (void) {
    eSTATUS_t result = Vector_Init (NULL, test_buffer_uint8, 8, sizeof (uint8_t), false);
    TEST_ASSERT_EQUAL (eSTATUS_FAILURE, result);
}

void test_VectorInit_NullBuffer (void) {
    eSTATUS_t result = Vector_Init (&test_vector_uint8, NULL, 8, sizeof (uint8_t), false);
    TEST_ASSERT_EQUAL (eSTATUS_FAILURE, result);
}

void test_VectorInit_ZeroElementSize (void) {
    eSTATUS_t result = Vector_Init (&test_vector_uint8, test_buffer_uint8, 8, 0, false);
    TEST_ASSERT_EQUAL (eSTATUS_FAILURE, result);
}

// Test basic vector operations
void test_VectorPushBack_ValidElement (void) {
    Vector_Init (&test_vector_uint8, test_buffer_uint8, 8, sizeof (uint8_t), false);

    uint8_t value    = 42;
    eSTATUS_t result = Vector_PushBack (&test_vector_uint8, &value);

    TEST_ASSERT_EQUAL (eSTATUS_SUCCESS, result);
    TEST_ASSERT_EQUAL (1, Vector_Size (&test_vector_uint8));
    TEST_ASSERT_FALSE (Vector_IsEmpty (&test_vector_uint8));
    TEST_ASSERT_FALSE (Vector_IsFull (&test_vector_uint8));

    uint8_t* pBack = (uint8_t*)Vector_Back (&test_vector_uint8);
    TEST_ASSERT_NOT_NULL (pBack);
    TEST_ASSERT_EQUAL (42, *pBack);
}

void test_VectorPushBack_NullVector (void) {
    uint8_t value    = 42;
    eSTATUS_t result = Vector_PushBack (NULL, &value);
    TEST_ASSERT_EQUAL (eSTATUS_FAILURE, result);
}

void test_VectorPushBack_NullElement (void) {
    Vector_Init (&test_vector_uint8, test_buffer_uint8, 8, sizeof (uint8_t), false);

    eSTATUS_t result = Vector_PushBack (&test_vector_uint8, NULL);
    TEST_ASSERT_EQUAL (eSTATUS_FAILURE, result);
}

void test_VectorPushBack_FullVector (void) {
    Vector_Init (&test_vector_uint8, test_buffer_uint8, 2, sizeof (uint8_t), false);

    uint8_t value1 = 1, value2 = 2, value3 = 3;

    TEST_ASSERT_EQUAL (eSTATUS_SUCCESS, Vector_PushBack (&test_vector_uint8, &value1));
    TEST_ASSERT_EQUAL (eSTATUS_SUCCESS, Vector_PushBack (&test_vector_uint8, &value2));
    TEST_ASSERT_TRUE (Vector_IsFull (&test_vector_uint8));

    // Should fail when full
    TEST_ASSERT_EQUAL (eSTATUS_FAILURE, Vector_PushBack (&test_vector_uint8, &value3));
    TEST_ASSERT_EQUAL (2, Vector_Size (&test_vector_uint8));
}

// Test Vector_PopBack
void test_VectorPopBack_ValidVector (void) {
    Vector_Init (&test_vector_uint8, test_buffer_uint8, 8, sizeof (uint8_t), false);

    uint8_t value = 42;
    Vector_PushBack (&test_vector_uint8, &value);

    uint8_t poppedValue;
    eSTATUS_t result = Vector_PopBack (&test_vector_uint8, &poppedValue);

    TEST_ASSERT_EQUAL (eSTATUS_SUCCESS, result);
    TEST_ASSERT_EQUAL (value, poppedValue);
    TEST_ASSERT_EQUAL (0, Vector_Size (&test_vector_uint8));
    TEST_ASSERT_TRUE (Vector_IsEmpty (&test_vector_uint8));
}

void test_VectorPopBack_NullVector (void) {
    uint8_t poppedValue;
    eSTATUS_t result = Vector_PopBack (NULL, &poppedValue);
    TEST_ASSERT_EQUAL (eSTATUS_FAILURE, result);
}

void test_VectorPopBack_EmptyVector (void) {
    Vector_Init (&test_vector_uint8, test_buffer_uint8, 8, sizeof (uint8_t), false);

    uint8_t poppedValue;
    eSTATUS_t result = Vector_PopBack (&test_vector_uint8, &poppedValue);
    TEST_ASSERT_EQUAL (eSTATUS_FAILURE, result);
}

void test_VectorPopBack_NullOutElement (void) {
    Vector_Init (&test_vector_uint8, test_buffer_uint8, 8, sizeof (uint8_t), false);

    uint8_t value = 42;
    Vector_PushBack (&test_vector_uint8, &value);

    // Should succeed even with null output element
    eSTATUS_t result = Vector_PopBack (&test_vector_uint8, NULL);
    TEST_ASSERT_EQUAL (eSTATUS_SUCCESS, result);
    TEST_ASSERT_EQUAL (0, Vector_Size (&test_vector_uint8));
}

// Test Vector_At
void test_VectorAt_ValidIndex (void) {
    Vector_Init (&test_vector_uint8, test_buffer_uint8, 8, sizeof (uint8_t), false);

    uint8_t values[] = { 10, 20, 30 };
    for (int i = 0; i < 3; i++) {
        Vector_PushBack (&test_vector_uint8, &values[i]);
    }

    uint8_t* pElement = (uint8_t*)Vector_At (&test_vector_uint8, 1);
    TEST_ASSERT_NOT_NULL (pElement);
    TEST_ASSERT_EQUAL (20, *pElement);
}

void test_VectorAt_InvalidIndex (void) {
    Vector_Init (&test_vector_uint8, test_buffer_uint8, 8, sizeof (uint8_t), false);

    uint8_t value = 42;
    Vector_PushBack (&test_vector_uint8, &value);

    void* pElement = Vector_At (&test_vector_uint8, 5); // Out of bounds
    TEST_ASSERT_NULL (pElement);
}

void test_VectorAt_NullVector (void) {
    void* pElement = Vector_At (NULL, 0);
    TEST_ASSERT_NULL (pElement);
}

// Test Vector_Front and Vector_Back
void test_VectorFront_ValidVector (void) {
    Vector_Init (&test_vector_uint8, test_buffer_uint8, 8, sizeof (uint8_t), false);

    uint8_t values[] = { 10, 20, 30 };
    for (int i = 0; i < 3; i++) {
        Vector_PushBack (&test_vector_uint8, &values[i]);
    }

    uint8_t* pFront = (uint8_t*)Vector_Front (&test_vector_uint8);
    TEST_ASSERT_NOT_NULL (pFront);
    TEST_ASSERT_EQUAL (10, *pFront);
}

void test_VectorBack_ValidVector (void) {
    Vector_Init (&test_vector_uint8, test_buffer_uint8, 8, sizeof (uint8_t), false);

    uint8_t values[] = { 10, 20, 30 };
    for (int i = 0; i < 3; i++) {
        Vector_PushBack (&test_vector_uint8, &values[i]);
    }

    uint8_t* pBack = (uint8_t*)Vector_Back (&test_vector_uint8);
    TEST_ASSERT_NOT_NULL (pBack);
    TEST_ASSERT_EQUAL (30, *pBack);
}

void test_VectorFrontBack_EmptyVector (void) {
    Vector_Init (&test_vector_uint8, test_buffer_uint8, 8, sizeof (uint8_t), false);

    void* pFront = Vector_Front (&test_vector_uint8);
    void* pBack  = Vector_Back (&test_vector_uint8);

    TEST_ASSERT_NULL (pFront);
    TEST_ASSERT_NULL (pBack);
}

void test_VectorFrontBack_NullVector (void) {
    void* pFront = Vector_Front (NULL);
    void* pBack  = Vector_Back (NULL);

    TEST_ASSERT_NULL (pFront);
    TEST_ASSERT_NULL (pBack);
}

// Test Vector_Insert
void test_VectorInsert_ValidIndex (void) {
    Vector_Init (&test_vector_uint8, test_buffer_uint8, 8, sizeof (uint8_t), false);

    uint8_t values[] = { 10, 30 };
    for (int i = 0; i < 2; i++) {
        Vector_PushBack (&test_vector_uint8, &values[i]);
    }

    uint8_t insertValue = 20;
    eSTATUS_t result    = Vector_Insert (&test_vector_uint8, 1, &insertValue);

    TEST_ASSERT_EQUAL (eSTATUS_SUCCESS, result);
    TEST_ASSERT_EQUAL (3, Vector_Size (&test_vector_uint8));

    uint8_t* pElement = (uint8_t*)Vector_At (&test_vector_uint8, 1);
    TEST_ASSERT_EQUAL (20, *pElement);

    pElement = (uint8_t*)Vector_At (&test_vector_uint8, 2);
    TEST_ASSERT_EQUAL (30, *pElement);
}

void test_VectorInsert_AtEnd (void) {
    Vector_Init (&test_vector_uint8, test_buffer_uint8, 8, sizeof (uint8_t), false);

    uint8_t value1 = 10;
    Vector_PushBack (&test_vector_uint8, &value1);

    uint8_t value2   = 20;
    eSTATUS_t result = Vector_Insert (&test_vector_uint8, 1, &value2);

    TEST_ASSERT_EQUAL (eSTATUS_SUCCESS, result);
    TEST_ASSERT_EQUAL (2, Vector_Size (&test_vector_uint8));

    uint8_t* pBack = (uint8_t*)Vector_Back (&test_vector_uint8);
    TEST_ASSERT_EQUAL (20, *pBack);
}

void test_VectorInsert_InvalidParameters (void) {
    Vector_Init (&test_vector_uint8, test_buffer_uint8, 8, sizeof (uint8_t), false);

    uint8_t value = 42;

    // Null vector
    TEST_ASSERT_EQUAL (eSTATUS_FAILURE, Vector_Insert (NULL, 0, &value));

    // Null element
    TEST_ASSERT_EQUAL (eSTATUS_FAILURE, Vector_Insert (&test_vector_uint8, 0, NULL));

    // Index out of bounds
    TEST_ASSERT_EQUAL (eSTATUS_FAILURE, Vector_Insert (&test_vector_uint8, 5, &value));
}

void test_VectorInsert_FullVector (void) {
    Vector_Init (&test_vector_uint8, test_buffer_uint8, 2, sizeof (uint8_t), false);

    uint8_t value1 = 1, value2 = 2, value3 = 3;
    Vector_PushBack (&test_vector_uint8, &value1);
    Vector_PushBack (&test_vector_uint8, &value2);

    eSTATUS_t result = Vector_Insert (&test_vector_uint8, 1, &value3);
    TEST_ASSERT_EQUAL (eSTATUS_FAILURE, result);
}

// Test Vector_Erase
void test_VectorErase_ValidIndex (void) {
    Vector_Init (&test_vector_uint8, test_buffer_uint8, 8, sizeof (uint8_t), false);

    uint8_t values[] = { 10, 20, 30 };
    for (int i = 0; i < 3; i++) {
        Vector_PushBack (&test_vector_uint8, &values[i]);
    }

    uint8_t erasedValue;
    eSTATUS_t result = Vector_Erase (&test_vector_uint8, 1, &erasedValue);

    TEST_ASSERT_EQUAL (eSTATUS_SUCCESS, result);
    TEST_ASSERT_EQUAL (20, erasedValue);
    TEST_ASSERT_EQUAL (2, Vector_Size (&test_vector_uint8));

    uint8_t* pElement = (uint8_t*)Vector_At (&test_vector_uint8, 1);
    TEST_ASSERT_EQUAL (30, *pElement);
}

void test_VectorErase_NullOutElement (void) {
    Vector_Init (&test_vector_uint8, test_buffer_uint8, 8, sizeof (uint8_t), false);

    uint8_t value = 42;
    Vector_PushBack (&test_vector_uint8, &value);

    eSTATUS_t result = Vector_Erase (&test_vector_uint8, 0, NULL);
    TEST_ASSERT_EQUAL (eSTATUS_SUCCESS, result);
    TEST_ASSERT_EQUAL (0, Vector_Size (&test_vector_uint8));
}

void test_VectorErase_InvalidParameters (void) {
    Vector_Init (&test_vector_uint8, test_buffer_uint8, 8, sizeof (uint8_t), false);

    uint8_t value = 42;
    Vector_PushBack (&test_vector_uint8, &value);

    uint8_t erasedValue;

    // Null vector
    TEST_ASSERT_EQUAL (eSTATUS_FAILURE, Vector_Erase (NULL, 0, &erasedValue));

    // Index out of bounds
    TEST_ASSERT_EQUAL (eSTATUS_FAILURE, Vector_Erase (&test_vector_uint8, 5, &erasedValue));
}

// Test Vector_Clear
void test_VectorClear_ValidVector (void) {
    Vector_Init (&test_vector_uint8, test_buffer_uint8, 8, sizeof (uint8_t), false);

    uint8_t values[] = { 10, 20, 30 };
    for (int i = 0; i < 3; i++) {
        Vector_PushBack (&test_vector_uint8, &values[i]);
    }

    Vector_Clear (&test_vector_uint8);

    TEST_ASSERT_EQUAL (0, Vector_Size (&test_vector_uint8));
    TEST_ASSERT_TRUE (Vector_IsEmpty (&test_vector_uint8));
    TEST_ASSERT_FALSE (Vector_IsFull (&test_vector_uint8));
}

void test_VectorClear_NullVector (void) {
    // Should not crash
    Vector_Clear (NULL);
}

// Test Vector_Resize
void test_VectorResize_GrowWithFill (void) {
    Vector_Init (&test_vector_uint8, test_buffer_uint8, 8, sizeof (uint8_t), false);

    uint8_t values[] = { 10, 20 };
    for (int i = 0; i < 2; i++) {
        Vector_PushBack (&test_vector_uint8, &values[i]);
    }

    uint8_t fillValue = 99;
    eSTATUS_t result  = Vector_Resize (&test_vector_uint8, 5, &fillValue);

    TEST_ASSERT_EQUAL (eSTATUS_SUCCESS, result);
    TEST_ASSERT_EQUAL (5, Vector_Size (&test_vector_uint8));

    // Check original elements
    TEST_ASSERT_EQUAL (10, *(uint8_t*)Vector_At (&test_vector_uint8, 0));
    TEST_ASSERT_EQUAL (20, *(uint8_t*)Vector_At (&test_vector_uint8, 1));

    // Check filled elements
    for (int i = 2; i < 5; i++) {
        TEST_ASSERT_EQUAL (99, *(uint8_t*)Vector_At (&test_vector_uint8, i));
    }
}

void test_VectorResize_Shrink (void) {
    Vector_Init (&test_vector_uint8, test_buffer_uint8, 8, sizeof (uint8_t), false);

    uint8_t values[] = { 10, 20, 30, 40 };
    for (int i = 0; i < 4; i++) {
        Vector_PushBack (&test_vector_uint8, &values[i]);
    }

    eSTATUS_t result = Vector_Resize (&test_vector_uint8, 2, NULL);

    TEST_ASSERT_EQUAL (eSTATUS_SUCCESS, result);
    TEST_ASSERT_EQUAL (2, Vector_Size (&test_vector_uint8));

    TEST_ASSERT_EQUAL (10, *(uint8_t*)Vector_At (&test_vector_uint8, 0));
    TEST_ASSERT_EQUAL (20, *(uint8_t*)Vector_At (&test_vector_uint8, 1));
}

void test_VectorResize_InvalidParameters (void) {
    Vector_Init (&test_vector_uint8, test_buffer_uint8, 8, sizeof (uint8_t), false);

    uint8_t fillValue = 99;

    // Null vector
    TEST_ASSERT_EQUAL (eSTATUS_FAILURE, Vector_Resize (NULL, 5, &fillValue));

    // Size exceeds capacity
    TEST_ASSERT_EQUAL (eSTATUS_FAILURE, Vector_Resize (&test_vector_uint8, 10, &fillValue));
}

// Test Vector_Data
void test_VectorData_ValidVector (void) {
    Vector_Init (&test_vector_uint8, test_buffer_uint8, 8, sizeof (uint8_t), false);

    void* pData = Vector_Data (&test_vector_uint8);
    TEST_ASSERT_NOT_NULL (pData);
    TEST_ASSERT_EQUAL (test_buffer_uint8, pData);
}

void test_VectorData_NullVector (void) {
    void* pData = Vector_Data (NULL);
    TEST_ASSERT_NULL (pData);
}

// Test with uint32_t data type
void test_VectorUint32 (void) {
    Vector_Init (&test_vector_uint32, test_buffer_uint32, 4, sizeof (uint32_t), false);

    uint32_t values[] = { 0x12345678, 0x9ABCDEF0, 0xFEDCBA98 };

    for (int i = 0; i < 3; i++) {
        TEST_ASSERT_EQUAL (eSTATUS_SUCCESS, Vector_PushBack (&test_vector_uint32, &values[i]));
    }

    TEST_ASSERT_EQUAL (3, Vector_Size (&test_vector_uint32));

    for (int i = 0; i < 3; i++) {
        uint32_t* pElement = (uint32_t*)Vector_At (&test_vector_uint32, i);
        TEST_ASSERT_NOT_NULL (pElement);
        TEST_ASSERT_EQUAL (values[i], *pElement);
    }
}

// Test with struct data type
void test_VectorStruct (void) {
    Vector_Init (&test_vector_struct, test_buffer_struct, 2, sizeof (TestStruct), false);

    TestStruct testData1 = { .id = 100, .value = 200, .flag = 1 };
    TestStruct testData2 = { .id = 300, .value = 400, .flag = 0 };

    TEST_ASSERT_EQUAL (eSTATUS_SUCCESS, Vector_PushBack (&test_vector_struct, &testData1));
    TEST_ASSERT_EQUAL (eSTATUS_SUCCESS, Vector_PushBack (&test_vector_struct, &testData2));

    TEST_ASSERT_EQUAL (2, Vector_Size (&test_vector_struct));
    TEST_ASSERT_TRUE (Vector_IsFull (&test_vector_struct));

    TestStruct* pStruct = (TestStruct*)Vector_At (&test_vector_struct, 0);
    TEST_ASSERT_NOT_NULL (pStruct);
    TEST_ASSERT_EQUAL (100, pStruct->id);
    TEST_ASSERT_EQUAL (200, pStruct->value);
    TEST_ASSERT_EQUAL (1, pStruct->flag);

    pStruct = (TestStruct*)Vector_At (&test_vector_struct, 1);
    TEST_ASSERT_NOT_NULL (pStruct);
    TEST_ASSERT_EQUAL (300, pStruct->id);
    TEST_ASSERT_EQUAL (400, pStruct->value);
    TEST_ASSERT_EQUAL (0, pStruct->flag);
}

// Test utility functions with null vector
void test_VectorUtilities_NullVector (void) {
    TEST_ASSERT_TRUE (Vector_IsEmpty (NULL));
    TEST_ASSERT_TRUE (Vector_IsFull (NULL));
    TEST_ASSERT_EQUAL (0, Vector_Size (NULL));
    TEST_ASSERT_EQUAL (0, Vector_Capacity (NULL));
}

// Test complete workflow
void test_VectorCompleteWorkflow (void) {
    Vector_Init (&test_vector_uint8, test_buffer_uint8, 4, sizeof (uint8_t), false);

    // Add elements
    uint8_t values[] = { 10, 20, 30 };
    for (int i = 0; i < 3; i++) {
        TEST_ASSERT_EQUAL (eSTATUS_SUCCESS, Vector_PushBack (&test_vector_uint8, &values[i]));
    }
    TEST_ASSERT_EQUAL (3, Vector_Size (&test_vector_uint8));

    // Insert element
    uint8_t insertValue = 15;
    TEST_ASSERT_EQUAL (eSTATUS_SUCCESS, Vector_Insert (&test_vector_uint8, 1, &insertValue));
    TEST_ASSERT_EQUAL (4, Vector_Size (&test_vector_uint8));
    TEST_ASSERT_TRUE (Vector_IsFull (&test_vector_uint8));

    // Verify order: [10, 15, 20, 30]
    uint8_t expected[] = { 10, 15, 20, 30 };
    for (int i = 0; i < 4; i++) {
        TEST_ASSERT_EQUAL (expected[i], *(uint8_t*)Vector_At (&test_vector_uint8, i));
    }

    // Erase element
    uint8_t erasedValue;
    TEST_ASSERT_EQUAL (eSTATUS_SUCCESS, Vector_Erase (&test_vector_uint8, 2, &erasedValue));
    TEST_ASSERT_EQUAL (20, erasedValue);
    TEST_ASSERT_EQUAL (3, Vector_Size (&test_vector_uint8));

    // Pop back
    uint8_t poppedValue;
    TEST_ASSERT_EQUAL (eSTATUS_SUCCESS, Vector_PopBack (&test_vector_uint8, &poppedValue));
    TEST_ASSERT_EQUAL (30, poppedValue);
    TEST_ASSERT_EQUAL (2, Vector_Size (&test_vector_uint8));

    // Final state: [10, 15]
    TEST_ASSERT_EQUAL (10, *(uint8_t*)Vector_Front (&test_vector_uint8));
    TEST_ASSERT_EQUAL (15, *(uint8_t*)Vector_Back (&test_vector_uint8));

    // Clear
    Vector_Clear (&test_vector_uint8);
    TEST_ASSERT_TRUE (Vector_IsEmpty (&test_vector_uint8));
}

int main (void) {
    UNITY_BEGIN ();

    RUN_TEST (test_VectorInit_ValidParameters);
    RUN_TEST (test_VectorInit_SharedVector);
    RUN_TEST (test_VectorInit_NullVector);
    RUN_TEST (test_VectorInit_NullBuffer);
    RUN_TEST (test_VectorInit_ZeroElementSize);

    RUN_TEST (test_VectorPushBack_ValidElement);
    RUN_TEST (test_VectorPushBack_NullVector);
    RUN_TEST (test_VectorPushBack_NullElement);
    RUN_TEST (test_VectorPushBack_FullVector);

    RUN_TEST (test_VectorPopBack_ValidVector);
    RUN_TEST (test_VectorPopBack_NullVector);
    RUN_TEST (test_VectorPopBack_EmptyVector);
    RUN_TEST (test_VectorPopBack_NullOutElement);

    RUN_TEST (test_VectorAt_ValidIndex);
    RUN_TEST (test_VectorAt_InvalidIndex);
    RUN_TEST (test_VectorAt_NullVector);

    RUN_TEST (test_VectorFront_ValidVector);
    RUN_TEST (test_VectorBack_ValidVector);
    RUN_TEST (test_VectorFrontBack_EmptyVector);
    RUN_TEST (test_VectorFrontBack_NullVector);

    RUN_TEST (test_VectorInsert_ValidIndex);
    RUN_TEST (test_VectorInsert_AtEnd);
    RUN_TEST (test_VectorInsert_InvalidParameters);
    RUN_TEST (test_VectorInsert_FullVector);

    RUN_TEST (test_VectorErase_ValidIndex);
    RUN_TEST (test_VectorErase_NullOutElement);
    RUN_TEST (test_VectorErase_InvalidParameters);

    RUN_TEST (test_VectorClear_ValidVector);
    RUN_TEST (test_VectorClear_NullVector);

    RUN_TEST (test_VectorResize_GrowWithFill);
    RUN_TEST (test_VectorResize_Shrink);
    RUN_TEST (test_VectorResize_InvalidParameters);

    RUN_TEST (test_VectorData_ValidVector);
    RUN_TEST (test_VectorData_NullVector);

    RUN_TEST (test_VectorUint32);
    RUN_TEST (test_VectorStruct);

    RUN_TEST (test_VectorUtilities_NullVector);
    RUN_TEST (test_VectorCompleteWorkflow);

    return UNITY_END ();
}