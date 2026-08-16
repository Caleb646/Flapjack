#include "core/core.h"
#include "hal.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

float ge_ScaledSystemCoreClock = 0.0F;

static void DWTInit (void);
static bool SanityCheckTimers (void);

eSTATUS_t CoreShared_Init (void) {

    DWTInit ();
    // Scale by 10Mhz for fDelayMicroseconds function
    ge_ScaledSystemCoreClock = (float)SystemCoreClock / 10000000.0F;

    if (SanityCheckTimers () == false) {
        return eSTATUS_FAILURE;
    }

    return eSTATUS_SUCCESS;
}

void CriticalErrorHandler (void) {

    __disable_irq ();
    __BKPT (1);
    while (1) {
    };
}

void __assert_func (const char* file, int line, const char* func, const char* failedexpr) {

    FJ_UNUSED (file);
    FJ_UNUSED (line);
    FJ_UNUSED (func);
    FJ_UNUSED (failedexpr);

    __BKPT (1);
    // asm volatile ("bkpt 1");
    CriticalErrorHandler ();
    while (1) {
    };
}

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
    if (ABS_F32 (fromMax - fromMin) < 0.0001F) {
        return toMin;
    }
    return toMin + ((v - fromMin) / (fromMax - fromMin)) * (toMax - toMin);
}

static void DWTInit (void) {

#ifndef UNIT_TEST
    // Enable core debug access and trace unit
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    // Enable DWT cycle counter
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    DWT->CYCCNT = 0;

#endif // UNIT_TEST
}

static bool SanityCheckTimers (void) {

    uint32_t const msDelay = 10;
    uint32_t tempStart     = GetMilliseconds ();
    HAL_Delay (msDelay);
    uint32_t tempEnd = GetMilliseconds ();

    if (tempEnd == tempStart || tempEnd < tempStart) {
        return false;
    }

    if ((tempEnd - tempStart) < msDelay - 2U || (tempEnd - tempStart) > msDelay + 2U) {
        return false;
    }

    uint32_t const usDelay = msDelay * 1000U;
    tempStart              = GetMicroseconds ();
    DelayMicroseconds (usDelay);
    tempEnd = GetMicroseconds ();

    if (tempEnd == tempStart || tempEnd < tempStart) {
        return false;
    }

    if ((tempEnd - tempStart) < usDelay - (2U * 1000U) || (tempEnd - tempStart) > usDelay + (2U * 1000U)) {
        return false;
    }

    return true;
}

uint32_t GetMilliseconds (void) {
    // return pdMS_TO_TICKS (xTaskGetTickCount ());
    return HAL_GetTick ();
}

uint32_t GetMicroseconds (void) {
    /*
     * Derived from DWT->CYCCNT rather than (HAL tick + SysTick->VAL).
     *
     * That pair cannot be read coherently under an RTOS. The millisecond tick
     * only advances when the SysTick ISR runs, and FreeRTOS masks interrupts for
     * every critical section, so SysTick can wrap and reload while the tick is
     * still stale. Combining them then produces a timestamp up to a millisecond
     * in the PAST, and no amount of re-reading the tick detects it because both
     * reads return the same stale value.
     *
     * Every caller takes an unsigned difference against a previous reading, so
     * one backwards step underflows to ~4295 seconds. Handed to the attitude
     * filter as dt, that integrates a single enormous step and leaves the
     * quaternion at an arbitrary orientation - measured at roughly one event
     * every five seconds, which is what destroyed the estimate in the SIL.
     * The same underflow corrupts the dt in pid.c and control.c.
     *
     * CYCCNT is one free-running register, so a single read is inherently
     * coherent. It wraps every 2^32 cycles (67 s at 64 MHz), too soon to divide
     * down directly - the quotient would wrap at ~67e6 instead of 2^32 and break
     * the callers' difference arithmetic - so accumulate whole microseconds and
     * carry the leftover cycles. This must be called at least once per 67 s or a
     * wrap goes unnoticed; the GNC loop calls it hundreds of times a second.
     */
#ifndef UNIT_TEST
    static uint32_t s_lastCycles = 0U;
    static uint32_t s_us         = 0U;

    uint32_t const cyclesPerUs = (SystemCoreClock / 1000000U);

    uint32_t const primask = __get_PRIMASK ();
    __disable_irq ();

    uint32_t const elapsed = DWT->CYCCNT - s_lastCycles; /* correct across wrap */
    uint32_t const us      = elapsed / cyclesPerUs;
    s_lastCycles += us * cyclesPerUs;                    /* keep the remainder */
    s_us += us;
    uint32_t const now = s_us;

    __set_PRIMASK (primask);
    return now;
#endif // UNIT_TEST
    return 0;
}

void Delay (uint32_t ms) {

    DelayMicroseconds (ms * 1000U);
}

void DelayMicroseconds (uint32_t us) {

#ifndef UNIT_TEST

    uint32_t usStart = GetMicroseconds ();
    while ((GetMicroseconds () - usStart) < us) {
        __NOP ();
    }

#endif // UNIT_TEST
}


// void fDelayMicroseconds (float us) {

// #ifndef UNIT_TEST

//     /*
//      * Example: 6.67 us
//      * time = 6.67 * 10 = 66.7
//      * cycles = 66.7 * (64MHz / 10MHz) = 66.7 * 6.4 = uint32_t(426.88 + 0.5)
//      * -> 427 cycles
//      * error = 427 (cycles) - 426.88 (exact cycles) = 0.12 cycles
//      */
//     uint32_t start  = DWT->CYCCNT;
//     uint32_t cycles = (uint32_t)(((us * 10.0F) * ge_ScaledSystemCoreClock) + 0.5F);
//     while ((DWT->CYCCNT - start) < cycles) {
//         __NOP ();
//     }

// #endif // UNIT_TEST
// }

/* Source --> Betaflight: https://github.com/betaflight/betaflight/blob/master/src/main/build/atomic.h */
void BasePriRestoreMem (uint8_t* val) {

#ifndef UNIT_TEST

    __set_BASEPRI (*val);

#endif // UNIT_TEST
}

// set BASEPRI_MAX, with global memory barrier, returns true
uint8_t BasePriSetMemRetVal (uint8_t prio) {

#ifndef UNIT_TEST

    __set_BASEPRI_MAX (prio);
    return 1;

#endif // UNIT_TEST
    return 1;
}
