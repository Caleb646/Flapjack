#ifndef TEST_RUNNER_H
#define TEST_RUNNER_H

#include "unity/unity.h"
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// Maximum number of tests that can be registered
#define MAX_TESTS 300

// Test function pointer type
typedef void (*test_func_t) (void);

// Test registration structure
typedef struct {
    const char* name;
    test_func_t func;
} test_registration_t;

// Global test registry
extern test_registration_t g_test_registry[MAX_TESTS];
extern size_t g_test_count;

void register_test (const char* name, test_func_t func);
void list_registered_tests (void);
test_func_t find_test_by_name (const char* name);
int run_test_by_name (const char* name);
int run_all_tests (void);
void register_all_tests (void);

#ifdef __cplusplus
}
#endif

#endif // TEST_RUNNER_H
