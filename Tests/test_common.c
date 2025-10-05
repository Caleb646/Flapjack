#include "common.h"
#include "unity/unity.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#ifndef UNIT_TEST
#error "UNIT_TEST should be defined in this file"
#endif

// Test data structures
typedef struct {
    int value;
    char name[8];
} TestStruct;

void setUp (void) {
    // This is run before EACH test
}

void tearDown (void) {
    // This is run after EACH test
}

// ============================================================================
// FOR_EACH Tests
// ============================================================================

void test_FOR_EACH_IntArray (void) {
    int testArray[] = { 10, 20, 30, 40, 50 };
    int sum         = 0;
    int count       = 0;

    FOR_EACH (int, testArray) {
        sum += *pElement;
        count++;
    }

    TEST_ASSERT_EQUAL (150, sum); // 10+20+30+40+50 = 150
    TEST_ASSERT_EQUAL (5, count);
}

void test_FOR_EACH_FloatArray (void) {
    float testArray[] = { 1.5F, 2.5F, 3.5F };
    float product     = 1.0F;
    int count         = 0;

    FOR_EACH (float, testArray) {
        product *= *pElement;
        count++;
    }

    TEST_ASSERT_FLOAT_WITHIN (0.001F, 13.125F, product); // 1.5 * 2.5 * 3.5 = 13.125
    TEST_ASSERT_EQUAL (3, count);
}

void test_FOR_EACH_StructArray (void) {
    TestStruct testArray[] = { { .value = 100, .name = "first" },
                               { .value = 200, .name = "second" },
                               { .value = 300, .name = "third" } };

    int totalValue = 0;
    int count      = 0;

    FOR_EACH (TestStruct, testArray) {
        totalValue += pElement->value;
        count++;

        // Verify we can access struct members
        TEST_ASSERT_TRUE (pElement->value > 0);
        TEST_ASSERT_TRUE (strlen (pElement->name) > 0);
    }

    TEST_ASSERT_EQUAL (600, totalValue); // 100+200+300 = 600
    TEST_ASSERT_EQUAL (3, count);
}

void test_FOR_EACH_SingleElement (void) {
    uint32_t testArray[] = { 42 };
    uint32_t value       = 0;
    int count            = 0;

    FOR_EACH (uint32_t, testArray) {
        value = *pElement;
        count++;
    }

    TEST_ASSERT_EQUAL (42, value);
    TEST_ASSERT_EQUAL (1, count);
}

void test_FOR_EACH_ModifyElements (void) {
    int testArray[] = { 1, 2, 3, 4, 5 };

    // Double all elements
    FOR_EACH (int, testArray) {
        *pElement *= 2;
    }

    // Verify modification
    TEST_ASSERT_EQUAL (2, testArray[0]);
    TEST_ASSERT_EQUAL (4, testArray[1]);
    TEST_ASSERT_EQUAL (6, testArray[2]);
    TEST_ASSERT_EQUAL (8, testArray[3]);
    TEST_ASSERT_EQUAL (10, testArray[4]);
}

void test_FOR_EACH_PointerAccess (void) {
    int testArray[]          = { 11, 22, 33 };
    int* expectedAddresses[] = { &testArray[0], &testArray[1], &testArray[2] };
    int index                = 0;

    FOR_EACH (int, testArray) {
        // Verify pointer points to correct element
        TEST_ASSERT_EQUAL_PTR (expectedAddresses[index], pElement);
        TEST_ASSERT_EQUAL (testArray[index], *pElement);
        index++;
    }

    TEST_ASSERT_EQUAL (3, index);
}

void test_FOR_EACH_Vec3fArray (void) {
    Vec3f vectors[] = { { .x = 1.0F, .y = 2.0F, .z = 3.0F }, { .x = 4.0F, .y = 5.0F, .z = 6.0F } };

    float sumX = 0.0F, sumY = 0.0F, sumZ = 0.0F;

    FOR_EACH (Vec3f, vectors) {
        sumX += pElement->x;
        sumY += pElement->y;
        sumZ += pElement->z;
    }

    TEST_ASSERT_FLOAT_WITHIN (0.001F, 5.0F, sumX); // 1.0 + 4.0 = 5.0
    TEST_ASSERT_FLOAT_WITHIN (0.001F, 7.0F, sumY); // 2.0 + 5.0 = 7.0
    TEST_ASSERT_FLOAT_WITHIN (0.001F, 9.0F, sumZ); // 3.0 + 6.0 = 9.0
}

// ============================================================================
// FOR_EACH_CONST Tests
// ============================================================================

void test_FOR_EACH_CONST_IntArray (void) {
    const int testArray[] = { 5, 10, 15, 20 };
    int sum               = 0;
    int count             = 0;

    FOR_EACH_CONST (int, testArray) {
        sum += *pElement;
        count++;
        // Verify pElement is const (this should compile)
        // *pElement = 0;  // This line would cause a compilation error
    }

    TEST_ASSERT_EQUAL (50, sum); // 5+10+15+20 = 50
    TEST_ASSERT_EQUAL (4, count);
}

void test_FOR_EACH_CONST_StructArray (void) {
    const TestStruct testArray[] = { { .value = 777, .name = "const1" }, { .value = 888, .name = "const2" } };

    int maxValue = 0;
    int count    = 0;

    FOR_EACH_CONST (TestStruct, testArray) {
        if (pElement->value > maxValue) {
            maxValue = pElement->value;
        }
        count++;

        // Verify we can read but not modify
        TEST_ASSERT_TRUE (pElement->value > 0);
        // pElement->value = 0;  // This would cause compilation error
    }

    TEST_ASSERT_EQUAL (888, maxValue);
    TEST_ASSERT_EQUAL (2, count);
}

void test_FOR_EACH_CONST_ReadOnlyAccess (void) {
    const float testArray[] = { 3.14F, 2.71F, 1.41F };
    float product           = 1.0F;

    FOR_EACH_CONST (float, testArray) {
        product *= (*pElement);
        // Verification that pElement points to correct memory
        TEST_ASSERT_TRUE (*pElement > 0.0F);
    }

    TEST_ASSERT_FLOAT_WITHIN (0.01F, 11.998F, product); // Approximately 3.14 * 2.71 * 1.41
}

void test_FOR_EACH_CONST_Vec4fArray (void) {
    const Vec4f quaternions[] = { { .q1 = 1.0F, .q2 = 0.0F, .q3 = 0.0F, .q4 = 0.0F }, // Identity quaternion
                                  { .q1 = 0.707F, .q2 = 0.707F, .q3 = 0.0F, .q4 = 0.0F } };

    float sumQ1 = 0.0F;
    int count   = 0;

    FOR_EACH_CONST (Vec4f, quaternions) {
        sumQ1 += pElement->q1;
        count++;

        // Verify union access works
        TEST_ASSERT_EQUAL (pElement->x, pElement->q1);
        TEST_ASSERT_EQUAL (pElement->y, pElement->q2);
    }

    TEST_ASSERT_FLOAT_WITHIN (0.001F, 1.707F, sumQ1);
    TEST_ASSERT_EQUAL (2, count);
}

// ============================================================================
// Edge Cases and Complex Scenarios
// ============================================================================

void test_FOR_EACH_NestedLoop (void) {
    int outerArray[] = { 1, 2, 3 };
    int innerArray[] = { 10, 20 };
    int total        = 0;

    FOR_EACH (int, outerArray) {
        int outerValue = *pElement;
        FOR_EACH (int, innerArray) {
            total += outerValue * (*pElement);
        }
    }

    // (1*10 + 1*20) + (2*10 + 2*20) + (3*10 + 3*20) = 30 + 60 + 90 = 180
    TEST_ASSERT_EQUAL (180, total);
}

void test_FOR_EACH_WithBreakAndContinue (void) {
    int testArray[] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 };
    int sum         = 0;
    int count       = 0;

    FOR_EACH (int, testArray) {
        count++;

        if (*pElement % 2 == 0) {
            continue; // Skip even numbers
        }

        sum += *pElement;
    }

    TEST_ASSERT_EQUAL (25, sum);   // 1 + 3 + 5 + 7 + 9 = 25
    TEST_ASSERT_EQUAL (10, count); // Should have processed 10 elements before breaking
}

void test_FOR_EACH_ArrayOfPointers (void) {
    int value1 = 100, value2 = 200, value3 = 300;
    int* ptrArray[] = { &value1, &value2, &value3 };
    int total       = 0;

    FOR_EACH (int*, ptrArray) {
        total += **pElement; // Dereference the pointer to pointer
    }

    TEST_ASSERT_EQUAL (600, total);
}

void test_FOR_EACH_LargeArray (void) {
#define LARGE_ARRAY_SIZE 100
    int largeArray[LARGE_ARRAY_SIZE];

    // Initialize array
    for (int i = 0; i < LARGE_ARRAY_SIZE; i++) {
        largeArray[i] = i + 1; // 1, 2, 3, ..., 100
    }

    int sum   = 0;
    int count = 0;

    FOR_EACH (int, largeArray) {
        sum += *pElement;
        count++;
    }

    TEST_ASSERT_EQUAL (5050, sum); // Sum of 1 to 100 = n(n+1)/2 = 100*101/2 = 5050
    TEST_ASSERT_EQUAL (LARGE_ARRAY_SIZE, count);
}

void test_FOR_EACH_TypeSafety (void) {
    // Test that the macro works with different types without casting issues
    uint8_t byteArray[]   = { 0x01, 0x02, 0x03 };
    uint16_t wordArray[]  = { 0x1000, 0x2000 };
    uint32_t dwordArray[] = { 0x10000000 };

    uint32_t byteSum = 0, wordSum = 0, dwordSum = 0;

    FOR_EACH (uint8_t, byteArray) {
        byteSum += *pElement;
    }

    FOR_EACH (uint16_t, wordArray) {
        wordSum += *pElement;
    }

    FOR_EACH (uint32_t, dwordArray) {
        dwordSum += *pElement;
    }

    TEST_ASSERT_EQUAL (6, byteSum);
    TEST_ASSERT_EQUAL (0x3000, wordSum);
    TEST_ASSERT_EQUAL (0x10000000, dwordSum);
}

int main (void) {
    UNITY_BEGIN ();

    // FOR_EACH Tests
    RUN_TEST (test_FOR_EACH_IntArray);
    RUN_TEST (test_FOR_EACH_FloatArray);
    RUN_TEST (test_FOR_EACH_StructArray);
    RUN_TEST (test_FOR_EACH_SingleElement);
    RUN_TEST (test_FOR_EACH_ModifyElements);
    RUN_TEST (test_FOR_EACH_PointerAccess);
    RUN_TEST (test_FOR_EACH_Vec3fArray);

    // FOR_EACH_CONST Tests
    RUN_TEST (test_FOR_EACH_CONST_IntArray);
    RUN_TEST (test_FOR_EACH_CONST_StructArray);
    RUN_TEST (test_FOR_EACH_CONST_ReadOnlyAccess);
    RUN_TEST (test_FOR_EACH_CONST_Vec4fArray);

    // Edge Cases and Complex Scenarios
    RUN_TEST (test_FOR_EACH_NestedLoop);
    RUN_TEST (test_FOR_EACH_WithBreakAndContinue);
    RUN_TEST (test_FOR_EACH_ArrayOfPointers);
    RUN_TEST (test_FOR_EACH_LargeArray);
    RUN_TEST (test_FOR_EACH_TypeSafety);

    return UNITY_END ();
}