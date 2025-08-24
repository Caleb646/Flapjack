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

uint32_t GetMilliseconds (void) {
    return pdMS_TO_TICKS (xTaskGetTickCount ());
}

uint32_t GetMicroseconds (void) {
    uint32_t msTime = GetMilliseconds ();
    if (SysTick->LOAD == SysTick->VAL) {
        return msTime * 1000U;
    }
    uint32_t usTime = (SysTick->LOAD - SysTick->VAL) / (SysTick->LOAD / 1000U);
    return msTime * 1000U + usTime;
}

void DelayMicroseconds (uint32_t us) {
    // uint32_t msTime = pdMS_TO_TICKS (xTaskGetTickCount ());
    uint32_t usStart   = GetMicroseconds ();
    uint32_t usCurrent = usStart;
    while ((usCurrent - usStart) < us) {
        usCurrent = GetMicroseconds ();
    }
}
