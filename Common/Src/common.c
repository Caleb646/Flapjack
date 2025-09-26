#include "common.h"
#include "hal.h"
#include <stdint.h>
#include <string.h>

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

static inline void DWTInit (void) {
    static BOOL_t dwtEnabled = FALSE;
    if (dwtEnabled == TRUE) {
        return;
    }
    // Enable core debug access and trace unit
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    // Enable DWT cycle counter
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    DWT->CYCCNT = 0;
}


uint32_t GetMilliseconds (void) {
    return pdMS_TO_TICKS (xTaskGetTickCount ());
}

uint32_t GetMicroseconds (void) {
    /* CTRL  -->  Offset: 0x000 (R/W)  SysTick Control and Status Register */
    /* LOAD  -->  Offset: 0x004 (R/W)  SysTick Reload Value Register */
    /* VAL   -->  Offset: 0x008 (R/W)  SysTick Current Value Register */
    /* CALIB -->  Offset: 0x00C (R/ )  SysTick Calibration Register */
    /* Systick->LOAD Initial Value -->  SYSTICK_CLOCK_HZ (64MHz) / 1000U ) - 1UL = 64,000 */
    uint32_t msTime = GetMilliseconds ();
#if MICRO_DELAY_USE_SYSTICK == 1
    if (SysTick->LOAD == SysTick->VAL) {
        return msTime * 1000U;
    }
    uint32_t usTime = (SysTick->LOAD - SysTick->VAL) / (SysTick->LOAD / 1000U);
    return msTime * 1000U + usTime;
#else
    DWTInit ();
    uint32_t usTime = DWT->CYCCNT / (SystemCoreClock / 1000000U);
    return msTime * 1000U + usTime;
#endif
}

void DelayMicroseconds (uint32_t us) {
    uint32_t usStart = GetMicroseconds ();
    while ((GetMicroseconds () - usStart) < us) {
        __NOP ();
    }
}

void fDelayMicroseconds (float us) {
    if (us < 0.0F) {
        return;
    }
    // Ensure DWT is enabled
    DWTInit ();
    /*
     * Example: 6.67 us
     * time = 6.67 * 10 = 66.7
     * cycles = 66.7 * (64MHz / 10MHz) = 66.7 * 6.4 = uint32_t(426.88 + 0.5)
     * -> 427 cycles
     * error = 427 (cycles) - 426.88 (exact cycles) = 0.12 cycles
     */
    float time = us * 10.0F;
    uint32_t cycles =
    (uint32_t)((time * (float)(SystemCoreClock / 10000000U)) + 0.5F);
    uint32_t start = DWT->CYCCNT;
    while ((DWT->CYCCNT - start) < cycles) {
        __NOP ();
    }
}
