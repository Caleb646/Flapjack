#include "common.h"
#include "hal.h"
#include <stdint.h>
#include <string.h>

// Global variable to store assertion information when printf is not
// available AssertionInfo_t AssertionInfo = { 0 };


void CriticalErrorHandler (void) {
    __disable_irq ();
    __BKPT (1);
    while (1) {
    };
}

void __assert_func (const char* file, int line, const char* func, const char* failedexpr) {
    __BKPT (1);
    // asm volatile ("bkpt 1");
    CriticalErrorHandler ();
    while (1) {
    };
}
