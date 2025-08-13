#ifndef TEST_RUNNER_H
#define TEST_RUNNER_H

#include "unity/unity.h"
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MAX_TESTS 500U

typedef void (*test_func_t) (void);
typedef struct {
    const char* name;
    test_func_t func;
} test_registration_t;

extern test_registration_t g_TestRegistry[MAX_TESTS];
extern size_t g_TestCount;

void RegisterTest (const char* name, test_func_t func);
void ListRegisteredTests (void);
test_func_t FindTestbyName (const char* name);
int RunTestbyName (const char* name);
int RunAllTests (void);
void RegisterAllTests (void);

#ifdef __cplusplus
}
#endif

#endif // TEST_RUNNER_H
