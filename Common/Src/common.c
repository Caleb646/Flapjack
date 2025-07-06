#include "common.h"
#include "hal.h"
#include <stdint.h>

// Global variable to store assertion information when printf is not
// available AssertionInfo_t AssertionInfo = { 0 };

int32_t clipi32 (int32_t v, int32_t lower, int32_t upper) {
    if (v < lower) {
        return lower;
    }
    if (v > upper) {
        return upper;
    }
    return v;
}

float clipf32 (float v, float lower, float upper) {
    if (v < lower) {
        return lower;
    }
    if (v > upper) {
        return upper;
    }
    return v;
}

float mapf32 (float v, float fromMin, float fromMax, float toMin, float toMax) {
    if (fromMax == fromMin) {
        return toMin;
    }
    return toMin + ((v - fromMin) / (fromMax - fromMin)) * (toMax - toMin);
}

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

// Microsecond delay function
// This is a simple loop-based implementation
// For more precise timing, DWT (Data Watchpoint and Trace) unit could be used
void DelayMicroseconds (uint32_t us) {
    // Approximate loop count for microsecond delay
    // This assumes system clock around 480 MHz
    // Each loop iteration takes approximately 4-5 cycles
    // So for 1 microsecond: 480 MHz / 1 MHz = 480 cycles
    // 480 cycles / 4 cycles per loop = ~120 loops per microsecond

    volatile int32_t loops  = us * 120;
    volatile uint32_t dummy = 0;

    while (loops-- > 0) {
        dummy++; // Prevent compiler optimization
    }
}
