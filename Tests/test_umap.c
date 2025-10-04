#include "mem/umap.h"
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

UMAP_DEFINE_STATIC (NAME, uint32_t, uint32_t, 8, false);

// Test buffer and umap instances
static UMapEntry_t test_entries_uint32[8];
static uint32_t test_keys_uint32[8];
static uint32_t test_values_uint32[8];
static UMap_t test_umap_uint32;

static UMapEntry_t test_entries_string[4];
static char test_keys_string[4][16];
static uint32_t test_values_string[4];
static UMap_t test_umap_string;

static UMapEntry_t test_entries_struct[2];
static uint32_t test_keys_struct[2];
static TestStruct test_values_struct[2];
static UMap_t test_umap_struct;

// Custom hash function for testing
static uint32_t test_simple_hash (void const* pKey, uint16_t keySize) {
    uint32_t hash         = 0;
    uint8_t const* pBytes = (uint8_t const*)pKey;
    for (size_t i = 0; i < keySize; ++i) {
        hash += pBytes[i];
    }
    return hash;
}

// Custom equality function for testing
static bool test_simple_equal (void const* pKey1, void const* pKey2, uint16_t keySize) {
    return memcmp (pKey1, pKey2, keySize) == 0;
}

void setUp (void) {
    memset (test_entries_uint32, 0, sizeof (test_entries_uint32));
    memset (test_keys_uint32, 0, sizeof (test_keys_uint32));
    memset (test_values_uint32, 0, sizeof (test_values_uint32));
    memset (test_entries_string, 0, sizeof (test_entries_string));
    memset (test_keys_string, 0, sizeof (test_keys_string));
    memset (test_values_string, 0, sizeof (test_values_string));
    memset (test_entries_struct, 0, sizeof (test_entries_struct));
    memset (test_keys_struct, 0, sizeof (test_keys_struct));
    memset (test_values_struct, 0, sizeof (test_values_struct));
}

void tearDown (void) {
    // Clean up after each test
}

// Test UMap_Init function
void test_UMapInit_ValidParameters (void) {

    bool result = UMap_Init (
    &test_umap_uint32,
    test_entries_uint32,
    test_keys_uint32,
    test_values_uint32,
    8,
    sizeof (uint32_t),
    sizeof (uint32_t),
    false
    );

    TEST_ASSERT_TRUE (result);
    TEST_ASSERT_EQUAL (8, UMap_Capacity (&test_umap_uint32));
    TEST_ASSERT_EQUAL (0, UMap_Size (&test_umap_uint32));
    TEST_ASSERT_TRUE (UMap_IsEmpty (&test_umap_uint32));
    TEST_ASSERT_FALSE (UMap_IsFull (&test_umap_uint32));
    TEST_ASSERT_FLOAT_WITHIN (0.01f, 0.0f, UMap_LoadFactor (&test_umap_uint32));
}

void test_UMapInit_NullUMap (void) {
    bool result = UMap_Init (
    NULL,
    test_entries_uint32,
    test_keys_uint32,
    test_values_uint32,
    8,
    sizeof (uint32_t),
    sizeof (uint32_t),
    false
    );
    TEST_ASSERT_FALSE (result);
}

void test_UMapInit_NullEntries (void) {
    bool result =
    UMap_Init (&test_umap_uint32, NULL, test_keys_uint32, test_values_uint32, 8, sizeof (uint32_t), sizeof (uint32_t), false);
    TEST_ASSERT_FALSE (result);
}

void test_UMapInit_NullKeyBuffer (void) {
    bool result = UMap_Init (
    &test_umap_uint32,
    test_entries_uint32,
    NULL,
    test_values_uint32,
    8,
    sizeof (uint32_t),
    sizeof (uint32_t),
    false
    );
    TEST_ASSERT_FALSE (result);
}

void test_UMapInit_NullValueBuffer (void) {
    bool result = UMap_Init (
    &test_umap_uint32,
    test_entries_uint32,
    test_keys_uint32,
    NULL,
    8,
    sizeof (uint32_t),
    sizeof (uint32_t),
    false
    );
    TEST_ASSERT_FALSE (result);
}

void test_UMapInit_ZeroCapacity (void) {
    bool result = UMap_Init (
    &test_umap_uint32,
    test_entries_uint32,
    test_keys_uint32,
    test_values_uint32,
    0,
    sizeof (uint32_t),
    sizeof (uint32_t),
    false
    );
    TEST_ASSERT_FALSE (result);
}

void test_UMapInit_ZeroKeySize (void) {
    bool result =
    UMap_Init (&test_umap_uint32, test_entries_uint32, test_keys_uint32, test_values_uint32, 8, 0, sizeof (uint32_t), false);
    TEST_ASSERT_FALSE (result);
}

void test_UMapInit_ZeroValueSize (void) {
    bool result = UMap_Init (
    &test_umap_uint32,
    test_entries_uint32,
    test_keys_uint32,
    test_values_uint32,
    8,
    sizeof (uint32_t),
    0,
    false
    );
    TEST_ASSERT_FALSE (result);
}

// Test UMap_InitWithFunctions
void test_UMapInitWithFunctions_ValidParameters (void) {
    bool result = UMap_InitWithFunctions (
    &test_umap_uint32,
    test_entries_uint32,
    test_keys_uint32,
    test_values_uint32,
    8,
    sizeof (uint32_t),
    sizeof (uint32_t),
    false,
    test_simple_hash,
    test_simple_equal
    );

    TEST_ASSERT_TRUE (result);
    TEST_ASSERT_EQUAL (8, UMap_Capacity (&test_umap_uint32));
    TEST_ASSERT_EQUAL (0, UMap_Size (&test_umap_uint32));
    TEST_ASSERT_TRUE (UMap_IsEmpty (&test_umap_uint32));
    TEST_ASSERT_FALSE (UMap_IsFull (&test_umap_uint32));
}

void test_UMapInitWithFunctions_NullHashFunction (void) {
    bool result = UMap_InitWithFunctions (
    &test_umap_uint32,
    test_entries_uint32,
    test_keys_uint32,
    test_values_uint32,
    8,
    sizeof (uint32_t),
    sizeof (uint32_t),
    false,
    NULL,
    test_simple_equal
    );
    TEST_ASSERT_FALSE (result);
}

void test_UMapInitWithFunctions_NullEqualFunction (void) {
    bool result = UMap_InitWithFunctions (
    &test_umap_uint32,
    test_entries_uint32,
    test_keys_uint32,
    test_values_uint32,
    8,
    sizeof (uint32_t),
    sizeof (uint32_t),
    false,
    test_simple_hash,
    NULL
    );
    TEST_ASSERT_FALSE (result);
}

// Test basic insert and find operations
void test_UMapInsert_SingleItem (void) {
    UMap_Init (
    &test_umap_uint32,
    test_entries_uint32,
    test_keys_uint32,
    test_values_uint32,
    8,
    sizeof (uint32_t),
    sizeof (uint32_t),
    false
    );

    uint32_t key   = 42;
    uint32_t value = 100;

    bool result = UMap_Insert (&test_umap_uint32, &key, &value);
    TEST_ASSERT_TRUE (result);
    TEST_ASSERT_EQUAL (1, UMap_Size (&test_umap_uint32));
    TEST_ASSERT_FALSE (UMap_IsEmpty (&test_umap_uint32));
    TEST_ASSERT_FALSE (UMap_IsFull (&test_umap_uint32));
    TEST_ASSERT_FLOAT_WITHIN (0.01F, 0.125F, UMap_LoadFactor (&test_umap_uint32));
}

void test_UMapInsert_MultipleItems (void) {
    UMap_Init (
    &test_umap_uint32,
    test_entries_uint32,
    test_keys_uint32,
    test_values_uint32,
    8,
    sizeof (uint32_t),
    sizeof (uint32_t),
    false
    );

    for (uint32_t i = 1; i <= 5; i++) {
        uint32_t key   = i;
        uint32_t value = i * 10;
        bool result    = UMap_Insert (&test_umap_uint32, &key, &value);
        TEST_ASSERT_TRUE (result);
        TEST_ASSERT_EQUAL (i, UMap_Size (&test_umap_uint32));
    }

    TEST_ASSERT_EQUAL (5, UMap_Size (&test_umap_uint32));
    TEST_ASSERT_FALSE (UMap_IsEmpty (&test_umap_uint32));
    TEST_ASSERT_FALSE (UMap_IsFull (&test_umap_uint32));
    TEST_ASSERT_FLOAT_WITHIN (0.01f, 0.625f, UMap_LoadFactor (&test_umap_uint32));
}

void test_UMapInsert_UpdateExistingKey (void) {
    UMap_Init (
    &test_umap_uint32,
    test_entries_uint32,
    test_keys_uint32,
    test_values_uint32,
    8,
    sizeof (uint32_t),
    sizeof (uint32_t),
    false
    );

    uint32_t key    = 42;
    uint32_t value1 = 100;
    uint32_t value2 = 200;

    // Insert initial value
    bool result = UMap_Insert (&test_umap_uint32, &key, &value1);
    TEST_ASSERT_TRUE (result);
    TEST_ASSERT_EQUAL (1, UMap_Size (&test_umap_uint32));

    // Update with new value
    result = UMap_Insert (&test_umap_uint32, &key, &value2);
    TEST_ASSERT_TRUE (result);
    TEST_ASSERT_EQUAL (1, UMap_Size (&test_umap_uint32)); // Size should not change

    // Verify updated value
    uint32_t retrieved_value;
    bool found = UMap_Find (&test_umap_uint32, &key, &retrieved_value);
    TEST_ASSERT_TRUE (found);
    TEST_ASSERT_EQUAL (value2, retrieved_value);
}

void test_UMapInsert_NullParameters (void) {
    UMap_Init (
    &test_umap_uint32,
    test_entries_uint32,
    test_keys_uint32,
    test_values_uint32,
    8,
    sizeof (uint32_t),
    sizeof (uint32_t),
    false
    );

    uint32_t key   = 42;
    uint32_t value = 100;

    // Test null umap
    bool result = UMap_Insert (NULL, &key, &value);
    TEST_ASSERT_FALSE (result);

    // Test null key
    result = UMap_Insert (&test_umap_uint32, NULL, &value);
    TEST_ASSERT_FALSE (result);

    // Test null value
    result = UMap_Insert (&test_umap_uint32, &key, NULL);
    TEST_ASSERT_FALSE (result);
}

void test_UMapInsert_FullMap (void) {
    UMap_Init (
    &test_umap_uint32,
    test_entries_uint32,
    test_keys_uint32,
    test_values_uint32,
    2,
    sizeof (uint32_t),
    sizeof (uint32_t),
    false
    ); // Small capacity

    // Fill the map
    for (uint32_t i = 1; i <= 2; i++) {
        uint32_t key   = i;
        uint32_t value = i * 10;
        bool result    = UMap_Insert (&test_umap_uint32, &key, &value);
        TEST_ASSERT_TRUE (result);
    }

    TEST_ASSERT_TRUE (UMap_IsFull (&test_umap_uint32));

    // Try to insert one more item
    uint32_t key   = 3;
    uint32_t value = 30;
    bool result    = UMap_Insert (&test_umap_uint32, &key, &value);
    TEST_ASSERT_FALSE (result);
    TEST_ASSERT_EQUAL (2, UMap_Size (&test_umap_uint32));
}

// Test find operations
void test_UMapFind_ExistingKey (void) {
    UMap_Init (
    &test_umap_uint32,
    test_entries_uint32,
    test_keys_uint32,
    test_values_uint32,
    8,
    sizeof (uint32_t),
    sizeof (uint32_t),
    false
    );

    uint32_t key   = 42;
    uint32_t value = 100;
    UMap_Insert (&test_umap_uint32, &key, &value);

    uint32_t retrieved_value;
    bool found = UMap_Find (&test_umap_uint32, &key, &retrieved_value);
    TEST_ASSERT_TRUE (found);
    TEST_ASSERT_EQUAL (value, retrieved_value);
}

void test_UMapFind_NonExistingKey (void) {
    UMap_Init (
    &test_umap_uint32,
    test_entries_uint32,
    test_keys_uint32,
    test_values_uint32,
    8,
    sizeof (uint32_t),
    sizeof (uint32_t),
    false
    );

    uint32_t key   = 42;
    uint32_t value = 100;
    UMap_Insert (&test_umap_uint32, &key, &value);

    uint32_t non_existing_key = 999;
    uint32_t retrieved_value;
    bool found = UMap_Find (&test_umap_uint32, &non_existing_key, &retrieved_value);
    TEST_ASSERT_FALSE (found);
}

void test_UMapFind_NullParameters (void) {
    UMap_Init (
    &test_umap_uint32,
    test_entries_uint32,
    test_keys_uint32,
    test_values_uint32,
    8,
    sizeof (uint32_t),
    sizeof (uint32_t),
    false
    );

    uint32_t key   = 42;
    uint32_t value = 100;
    uint32_t retrieved_value;

    // Test null umap
    bool found = UMap_Find (NULL, &key, &retrieved_value);
    TEST_ASSERT_FALSE (found);

    // Test null key
    found = UMap_Find (&test_umap_uint32, NULL, &retrieved_value);
    TEST_ASSERT_FALSE (found);

    // Test null output value
    found = UMap_Find (&test_umap_uint32, &key, NULL);
    TEST_ASSERT_FALSE (found);
}

// Test UMap_FindPtr
void test_UMapFindPtr_ExistingKey (void) {
    UMap_Init (
    &test_umap_uint32,
    test_entries_uint32,
    test_keys_uint32,
    test_values_uint32,
    8,
    sizeof (uint32_t),
    sizeof (uint32_t),
    false
    );

    uint32_t key   = 42;
    uint32_t value = 100;
    UMap_Insert (&test_umap_uint32, &key, &value);

    uint32_t* pValue = (uint32_t*)UMap_FindPtr (&test_umap_uint32, &key);
    TEST_ASSERT_NOT_NULL (pValue);
    TEST_ASSERT_EQUAL (value, *pValue);

    // Test that we can modify through the pointer
    *pValue = 200;
    uint32_t retrieved_value;
    bool found = UMap_Find (&test_umap_uint32, &key, &retrieved_value);
    TEST_ASSERT_TRUE (found);
    TEST_ASSERT_EQUAL (200, retrieved_value);
}

void test_UMapFindPtr_NonExistingKey (void) {
    UMap_Init (
    &test_umap_uint32,
    test_entries_uint32,
    test_keys_uint32,
    test_values_uint32,
    8,
    sizeof (uint32_t),
    sizeof (uint32_t),
    false
    );

    uint32_t key   = 42;
    uint32_t value = 100;
    UMap_Insert (&test_umap_uint32, &key, &value);

    uint32_t non_existing_key = 999;
    void* pValue = UMap_FindPtr (&test_umap_uint32, &non_existing_key);
    TEST_ASSERT_NULL (pValue);
}

void test_UMapFindPtr_NullParameters (void) {
    UMap_Init (
    &test_umap_uint32,
    test_entries_uint32,
    test_keys_uint32,
    test_values_uint32,
    8,
    sizeof (uint32_t),
    sizeof (uint32_t),
    false
    );

    uint32_t key = 42;

    // Test null umap
    void* pValue = UMap_FindPtr (NULL, &key);
    TEST_ASSERT_NULL (pValue);

    // Test null key
    pValue = UMap_FindPtr (&test_umap_uint32, NULL);
    TEST_ASSERT_NULL (pValue);
}

// Test UMap_Contains
void test_UMapContains_ExistingKey (void) {
    UMap_Init (
    &test_umap_uint32,
    test_entries_uint32,
    test_keys_uint32,
    test_values_uint32,
    8,
    sizeof (uint32_t),
    sizeof (uint32_t),
    false
    );

    uint32_t key   = 42;
    uint32_t value = 100;
    UMap_Insert (&test_umap_uint32, &key, &value);

    bool contains = UMap_Contains (&test_umap_uint32, &key);
    TEST_ASSERT_TRUE (contains);
}

void test_UMapContains_NonExistingKey (void) {
    UMap_Init (
    &test_umap_uint32,
    test_entries_uint32,
    test_keys_uint32,
    test_values_uint32,
    8,
    sizeof (uint32_t),
    sizeof (uint32_t),
    false
    );

    uint32_t key   = 42;
    uint32_t value = 100;
    UMap_Insert (&test_umap_uint32, &key, &value);

    uint32_t non_existing_key = 999;
    bool contains = UMap_Contains (&test_umap_uint32, &non_existing_key);
    TEST_ASSERT_FALSE (contains);
}

void test_UMapContains_NullParameters (void) {
    UMap_Init (
    &test_umap_uint32,
    test_entries_uint32,
    test_keys_uint32,
    test_values_uint32,
    8,
    sizeof (uint32_t),
    sizeof (uint32_t),
    false
    );

    uint32_t key = 42;

    // Test null umap
    bool contains = UMap_Contains (NULL, &key);
    TEST_ASSERT_FALSE (contains);

    // Test null key
    contains = UMap_Contains (&test_umap_uint32, NULL);
    TEST_ASSERT_FALSE (contains);
}

// Test UMap_Clear
void test_UMapClear_WithItems (void) {
    UMap_Init (
    &test_umap_uint32,
    test_entries_uint32,
    test_keys_uint32,
    test_values_uint32,
    8,
    sizeof (uint32_t),
    sizeof (uint32_t),
    false
    );

    // Add some items
    for (uint32_t i = 1; i <= 3; i++) {
        uint32_t key   = i;
        uint32_t value = i * 10;
        bool success   = UMap_Insert (&test_umap_uint32, &key, &value);
        TEST_ASSERT_TRUE (success);
    }

    TEST_ASSERT_EQUAL (3, UMap_Size (&test_umap_uint32));
    TEST_ASSERT_FALSE (UMap_IsEmpty (&test_umap_uint32));

    UMap_Clear (&test_umap_uint32);

    TEST_ASSERT_EQUAL (0, UMap_Size (&test_umap_uint32));
    TEST_ASSERT_TRUE (UMap_IsEmpty (&test_umap_uint32));
    TEST_ASSERT_FLOAT_WITHIN (0.01f, 0.0f, UMap_LoadFactor (&test_umap_uint32));

    // Verify that we can insert again after clearing
    uint32_t key   = 42;
    uint32_t value = 100;
    bool result    = UMap_Insert (&test_umap_uint32, &key, &value);
    TEST_ASSERT_TRUE (result);
    TEST_ASSERT_EQUAL (1, UMap_Size (&test_umap_uint32));
}

void test_UMapClear_EmptyMap (void) {
    UMap_Init (
    &test_umap_uint32,
    test_entries_uint32,
    test_keys_uint32,
    test_values_uint32,
    8,
    sizeof (uint32_t),
    sizeof (uint32_t),
    false
    );

    UMap_Clear (&test_umap_uint32);

    TEST_ASSERT_EQUAL (0, UMap_Size (&test_umap_uint32));
    TEST_ASSERT_TRUE (UMap_IsEmpty (&test_umap_uint32));
}

void test_UMapClear_NullUMap (void) {
    // Should not crash
    UMap_Clear (NULL);
    TEST_ASSERT_TRUE (true); // If we get here, it didn't crash
}

// Test with string keys
void test_UMapStringKeys_InsertAndFind (void) {
    UMap_Init (&test_umap_string, test_entries_string, test_keys_string, test_values_string, 4, 16, sizeof (uint32_t), false);

    char key1[]     = "hello";
    char key2[]     = "world";
    uint32_t value1 = 123;
    uint32_t value2 = 456;

    // Insert items
    bool result = UMap_Insert (&test_umap_string, key1, &value1);
    TEST_ASSERT_TRUE (result);

    result = UMap_Insert (&test_umap_string, key2, &value2);
    TEST_ASSERT_TRUE (result);

    TEST_ASSERT_EQUAL (2, UMap_Size (&test_umap_string));

    // Find items
    uint32_t retrieved_value;
    bool found = UMap_Find (&test_umap_string, key1, &retrieved_value);
    TEST_ASSERT_TRUE (found);
    TEST_ASSERT_EQUAL (value1, retrieved_value);

    found = UMap_Find (&test_umap_string, key2, &retrieved_value);
    TEST_ASSERT_TRUE (found);
    TEST_ASSERT_EQUAL (value2, retrieved_value);

    // Test contains
    TEST_ASSERT_TRUE (UMap_Contains (&test_umap_string, key1));
    TEST_ASSERT_TRUE (UMap_Contains (&test_umap_string, key2));

    char non_existing_key[] = "test";
    TEST_ASSERT_FALSE (UMap_Contains (&test_umap_string, non_existing_key));
}

// Test with struct values
void test_UMapStructValues_InsertAndFind (void) {
    UMap_Init (
    &test_umap_struct,
    test_entries_struct,
    test_keys_struct,
    test_values_struct,
    2,
    sizeof (uint32_t),
    sizeof (TestStruct),
    false
    );

    uint32_t key1     = 1;
    uint32_t key2     = 2;
    TestStruct value1 = { 100, 200, 1 };
    TestStruct value2 = { 300, 400, 0 };

    // Insert items
    bool result = UMap_Insert (&test_umap_struct, &key1, &value1);
    TEST_ASSERT_TRUE (result);

    result = UMap_Insert (&test_umap_struct, &key2, &value2);
    TEST_ASSERT_TRUE (result);

    TEST_ASSERT_EQUAL (2, UMap_Size (&test_umap_struct));

    // Find items
    TestStruct retrieved_value;
    bool found = UMap_Find (&test_umap_struct, &key1, &retrieved_value);
    TEST_ASSERT_TRUE (found);
    TEST_ASSERT_EQUAL (value1.id, retrieved_value.id);
    TEST_ASSERT_EQUAL (value1.value, retrieved_value.value);
    TEST_ASSERT_EQUAL (value1.flag, retrieved_value.flag);

    found = UMap_Find (&test_umap_struct, &key2, &retrieved_value);
    TEST_ASSERT_TRUE (found);
    TEST_ASSERT_EQUAL (value2.id, retrieved_value.id);
    TEST_ASSERT_EQUAL (value2.value, retrieved_value.value);
    TEST_ASSERT_EQUAL (value2.flag, retrieved_value.flag);

    // Test FindPtr with struct
    TestStruct* pValue = (TestStruct*)UMap_FindPtr (&test_umap_struct, &key1);
    TEST_ASSERT_NOT_NULL (pValue);
    TEST_ASSERT_EQUAL (value1.id, pValue->id);
    TEST_ASSERT_EQUAL (value1.value, pValue->value);
    TEST_ASSERT_EQUAL (value1.flag, pValue->flag);
}

// Test hash collision handling (linear probing)
void test_UMapHashCollision_LinearProbing (void) {
    // Use a small capacity to force collisions
    UMap_Init (
    &test_umap_uint32,
    test_entries_uint32,
    test_keys_uint32,
    test_values_uint32,
    4,
    sizeof (uint32_t),
    sizeof (uint32_t),
    false
    );

    // Insert keys that will likely cause hash collisions in a small map
    uint32_t keys[] = { 1, 5, 9, 13 }; // These may collide depending on hash function
    uint32_t values[] = { 10, 50, 90, 130 };

    // Insert all items
    for (int i = 0; i < 4; i++) {
        bool result = UMap_Insert (&test_umap_uint32, &keys[i], &values[i]);
        TEST_ASSERT_TRUE (result);
    }

    TEST_ASSERT_EQUAL (4, UMap_Size (&test_umap_uint32));
    TEST_ASSERT_TRUE (UMap_IsFull (&test_umap_uint32));

    // Verify all items can be found
    for (int i = 0; i < 4; i++) {
        uint32_t retrieved_value;
        bool found = UMap_Find (&test_umap_uint32, &keys[i], &retrieved_value);
        TEST_ASSERT_TRUE (found);
        TEST_ASSERT_EQUAL (values[i], retrieved_value);
    }
}

// Test edge cases for query functions with NULL parameters
void test_UMapQueryFunctions_NullParameters (void) {
    // Test IsEmpty with NULL
    TEST_ASSERT_TRUE (UMap_IsEmpty (NULL));

    // Test IsFull with NULL
    TEST_ASSERT_TRUE (UMap_IsFull (NULL));

    // Test Size with NULL
    TEST_ASSERT_EQUAL (0, UMap_Size (NULL));

    // Test Capacity with NULL
    TEST_ASSERT_EQUAL (0, UMap_Capacity (NULL));

    // Test LoadFactor with NULL
    TEST_ASSERT_FLOAT_WITHIN (0.01f, 1.0f, UMap_LoadFactor (NULL));
}

// Test default hash function
void test_UMapDefaultHash_ValidInput (void) {
    uint32_t key  = 0x12345678;
    uint32_t hash = UMap_DefaultHash (&key, sizeof (key));
    TEST_ASSERT_NOT_EQUAL (0, hash);

    char str_key[]    = "test";
    uint32_t str_hash = UMap_DefaultHash (str_key, strlen (str_key));
    TEST_ASSERT_NOT_EQUAL (0, str_hash);

    // Same input should produce same hash
    uint32_t hash2 = UMap_DefaultHash (&key, sizeof (key));
    TEST_ASSERT_EQUAL (hash, hash2);
}

void test_UMapDefaultHash_NullInput (void) {
    uint32_t hash = UMap_DefaultHash (NULL, 4);
    TEST_ASSERT_EQUAL (0, hash);
}

void test_UMapDefaultHash_ZeroSize (void) {
    uint32_t key  = 0x12345678;
    uint32_t hash = UMap_DefaultHash (&key, 0);
    TEST_ASSERT_EQUAL (0, hash);
}

// Test default equality function
void test_UMapDefaultEqual_ValidInput (void) {
    uint32_t key1 = 0x12345678;
    uint32_t key2 = 0x12345678;
    uint32_t key3 = 0x87654321;

    bool equal = UMap_DefaultEqual (&key1, &key2, sizeof (key1));
    TEST_ASSERT_TRUE (equal);

    equal = UMap_DefaultEqual (&key1, &key3, sizeof (key1));
    TEST_ASSERT_FALSE (equal);

    char str1[] = "test";
    char str2[] = "test";
    char str3[] = "different";

    equal = UMap_DefaultEqual (str1, str2, strlen (str1));
    TEST_ASSERT_TRUE (equal);

    equal = UMap_DefaultEqual (str1, str3, strlen (str1));
    TEST_ASSERT_FALSE (equal);
}

void test_UMapDefaultEqual_NullInput (void) {
    uint32_t key = 0x12345678;

    bool equal = UMap_DefaultEqual (NULL, &key, sizeof (key));
    TEST_ASSERT_FALSE (equal);

    equal = UMap_DefaultEqual (&key, NULL, sizeof (key));
    TEST_ASSERT_FALSE (equal);

    equal = UMap_DefaultEqual (NULL, NULL, sizeof (key));
    TEST_ASSERT_FALSE (equal);
}

// Test custom hash and equality functions
void test_UMapCustomFunctions_SimpleHash (void) {
    UMap_InitWithFunctions (
    &test_umap_uint32,
    test_entries_uint32,
    test_keys_uint32,
    test_values_uint32,
    8,
    sizeof (uint32_t),
    sizeof (uint32_t),
    false,
    test_simple_hash,
    test_simple_equal
    );

    uint32_t key   = 42;
    uint32_t value = 100;

    bool result = UMap_Insert (&test_umap_uint32, &key, &value);
    TEST_ASSERT_TRUE (result);

    uint32_t retrieved_value;
    bool found = UMap_Find (&test_umap_uint32, &key, &retrieved_value);
    TEST_ASSERT_TRUE (found);
    TEST_ASSERT_EQUAL (value, retrieved_value);
}

int main (void) {
    UNITY_BEGIN ();

    // Basic initialization tests
    RUN_TEST (test_UMapInit_ValidParameters);
    RUN_TEST (test_UMapInit_NullUMap);
    RUN_TEST (test_UMapInit_NullEntries);
    RUN_TEST (test_UMapInit_NullKeyBuffer);
    RUN_TEST (test_UMapInit_NullValueBuffer);
    RUN_TEST (test_UMapInit_ZeroCapacity);
    RUN_TEST (test_UMapInit_ZeroKeySize);
    RUN_TEST (test_UMapInit_ZeroValueSize);

    // Initialization with custom functions
    RUN_TEST (test_UMapInitWithFunctions_ValidParameters);
    RUN_TEST (test_UMapInitWithFunctions_NullHashFunction);
    RUN_TEST (test_UMapInitWithFunctions_NullEqualFunction);

    // Insert operations
    RUN_TEST (test_UMapInsert_SingleItem);
    RUN_TEST (test_UMapInsert_MultipleItems);
    RUN_TEST (test_UMapInsert_UpdateExistingKey);
    RUN_TEST (test_UMapInsert_NullParameters);
    RUN_TEST (test_UMapInsert_FullMap);

    // Find operations
    RUN_TEST (test_UMapFind_ExistingKey);
    RUN_TEST (test_UMapFind_NonExistingKey);
    RUN_TEST (test_UMapFind_NullParameters);

    // FindPtr operations
    RUN_TEST (test_UMapFindPtr_ExistingKey);
    RUN_TEST (test_UMapFindPtr_NonExistingKey);
    RUN_TEST (test_UMapFindPtr_NullParameters);

    // Contains operations
    RUN_TEST (test_UMapContains_ExistingKey);
    RUN_TEST (test_UMapContains_NonExistingKey);
    RUN_TEST (test_UMapContains_NullParameters);

    // Clear operations
    RUN_TEST (test_UMapClear_WithItems);
    RUN_TEST (test_UMapClear_EmptyMap);
    RUN_TEST (test_UMapClear_NullUMap);

    // Different data types
    RUN_TEST (test_UMapStringKeys_InsertAndFind);
    RUN_TEST (test_UMapStructValues_InsertAndFind);

    // Collision handling
    RUN_TEST (test_UMapHashCollision_LinearProbing);

    // Edge cases
    RUN_TEST (test_UMapQueryFunctions_NullParameters);

    // Default functions
    RUN_TEST (test_UMapDefaultHash_ValidInput);
    RUN_TEST (test_UMapDefaultHash_NullInput);
    RUN_TEST (test_UMapDefaultHash_ZeroSize);
    RUN_TEST (test_UMapDefaultEqual_ValidInput);
    RUN_TEST (test_UMapDefaultEqual_NullInput);

    // Custom functions
    RUN_TEST (test_UMapCustomFunctions_SimpleHash);

    return UNITY_END ();
}