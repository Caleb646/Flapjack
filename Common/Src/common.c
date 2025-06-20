#include "common.h"
#include "cmsis_os.h"
#include "stm32h7xx.h"

int32_t clipi32 (int32_t v, int32_t lower, int32_t upper) {
    if (v < lower)
        return lower;
    else if (v > upper)
        return upper;
    else
        return v;
}

float clipf32 (float v, float lower, float upper) {
    if (v < lower)
        return lower;
    else if (v > upper)
        return upper;
    else
        return v;
}

float mapf32 (float v, float fromMin, float fromMax, float toMin, float toMax) {
    if (fromMax == fromMin)
        return toMin;
    return toMin + ((v - fromMin) / (fromMax - fromMin)) * (toMax - toMin);
}

void CriticalErrorHandler (void) {
    __disable_irq ();
    __BKPT (1);
    while (1);
}

void __assert_func (const char* file, int line, const char* func, const char* failedexpr) {
    __BKPT (1);
    // asm volatile ("bkpt 1");
    CriticalErrorHandler ();
    while (1);
}


