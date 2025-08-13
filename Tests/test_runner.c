#include "test_runner.h"
#include "unity/unity.h"
#include <stdio.h>
#include <string.h>

#ifndef UNIT_TEST
#error "UNIT_TEST should be defined in this file"
#endif

test_registration_t g_TestRegistry[MAX_TESTS];
size_t g_TestCount = 0;

void RegisterTest (const char* name, test_func_t func) {
    if (name == NULL) {
        printf ("ERROR: Cannot register test with NULL name\n");
        return;
    }

    if (func == NULL) {
        printf ("ERROR: Cannot register test '%s' with NULL function pointer\n", name);
        return;
    }

    if (g_TestCount >= MAX_TESTS) {
        printf ("ERROR: Test registry is full (max %d tests). Cannot register '%s'\n", MAX_TESTS, name);
        return;
    }

    for (size_t i = 0; i < g_TestCount; ++i) {
        if (g_TestRegistry[i].name != NULL &&
            strcmp (g_TestRegistry[i].name, name) == 0) {
            printf ("WARNING: Test '%s' already registered, overwriting\n", name);
            g_TestRegistry[i].func = func;
            return;
        }
    }

    // Register the test
    g_TestRegistry[g_TestCount].name = name;
    g_TestRegistry[g_TestCount].func = func;
    g_TestCount++;
}

int RunAllTests (void) {
    printf ("Running %zu registered tests...\n", g_TestCount);
    for (size_t i = 0; i < g_TestCount; i++) {
        printf (
        "Running test %zu/%zu: %s\n",
        i + 1,
        g_TestCount,
        g_TestRegistry[i].name
        );
        RUN_TEST (g_TestRegistry[i].func);
    }

    return 0;
}

void setUp (void) {
}

void tearDown (void) {
}

void ListRegisteredTests (void) {
    printf ("=== Registered Tests (%zu total) ===\n", g_TestCount);
    for (size_t i = 0; i < g_TestCount; i++) {
        printf ("%zu. %s\n", i + 1, g_TestRegistry[i].name);
    }
    printf ("=====================================\n");
}

test_func_t FindTestbyName (const char* name) {
    if (name == NULL) {
        return NULL;
    }

    for (size_t i = 0; i < g_TestCount; i++) {
        if (g_TestRegistry[i].name != NULL &&
            strcmp (g_TestRegistry[i].name, name) == 0) {
            return g_TestRegistry[i].func;
        }
    }
    return NULL;
}

int RunTestbyName (const char* name) {
    test_func_t func = FindTestbyName (name);
    if (func == NULL) {
        printf ("ERROR: Test '%s' not found\n", name);
        return -1;
    }

    printf ("Running test: %s\n", name);
    RUN_TEST (func);
    return 0;
}

int main (void) {
    RegisterAllTests ();
    UNITY_BEGIN ();
    RunAllTests ();
    return UNITY_END ();
}
