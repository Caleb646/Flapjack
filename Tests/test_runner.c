#include "test_runner.h"
#include "unity/unity.h"
#include <stdio.h>
#include <string.h>

#ifndef UNIT_TEST
#error "UNIT_TEST should be defined in this file"
#endif

// Global test registry
test_registration_t g_test_registry[MAX_TESTS];
size_t g_test_count = 0;

// Test registration function
void register_test (const char* name, test_func_t func) {
    // Validate inputs
    if (name == NULL) {
        printf ("ERROR: Cannot register test with NULL name\n");
        return;
    }

    if (func == NULL) {
        printf ("ERROR: Cannot register test '%s' with NULL function pointer\n", name);
        return;
    }

    // Check if we have space for more tests
    if (g_test_count >= MAX_TESTS) {
        printf ("ERROR: Test registry is full (max %d tests). Cannot register '%s'\n", MAX_TESTS, name);
        return;
    }

    // Check for duplicate test names
    for (size_t i = 0; i < g_test_count; i++) {
        if (g_test_registry[i].name != NULL && strcmp (g_test_registry[i].name, name) == 0) {
            printf ("WARNING: Test '%s' already registered, overwriting\n", name);
            // Overwrite the existing registration
            g_test_registry[i].func = func;
            return;
        }
    }

    // Register the test
    g_test_registry[g_test_count].name = name;
    g_test_registry[g_test_count].func = func;
    g_test_count++;
}

// Run all registered tests
int run_all_tests (void) {
    printf ("Running %zu registered tests...\n", g_test_count);
    for (size_t i = 0; i < g_test_count; i++) {
        printf (
        "Running test %zu/%zu: %s\n", i + 1, g_test_count,
        g_test_registry[i].name);
        RUN_TEST (g_test_registry[i].func);
    }

    return 0;
}

void setUp (void) {
    // Setup function called before each test
}

void tearDown (void) {
    // Teardown function called after each test
}

// Helper function to list all registered tests
void list_registered_tests (void) {
    printf ("=== Registered Tests (%zu total) ===\n", g_test_count);
    for (size_t i = 0; i < g_test_count; i++) {
        printf ("%zu. %s\n", i + 1, g_test_registry[i].name);
    }
    printf ("=====================================\n");
}

// Helper function to find a test by name
test_func_t find_test_by_name (const char* name) {
    if (name == NULL) {
        return NULL;
    }

    for (size_t i = 0; i < g_test_count; i++) {
        if (g_test_registry[i].name != NULL && strcmp (g_test_registry[i].name, name) == 0) {
            return g_test_registry[i].func;
        }
    }
    return NULL;
}

// Helper function to run a specific test by name
int run_test_by_name (const char* name) {
    test_func_t func = find_test_by_name (name);
    if (func == NULL) {
        printf ("ERROR: Test '%s' not found\n", name);
        return -1;
    }

    printf ("Running test: %s\n", name);
    RUN_TEST (func);
    return 0;
}

int main (void) {
    // Register all tests
    register_all_tests ();

    UNITY_BEGIN ();

    // Run all registered tests
    run_all_tests ();

    return UNITY_END ();
}
