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
    if ((fromMin - fromMax) < 0.0001F) {
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
    /* CTRL  -->  Offset: 0x000 (R/W)  SysTick Control and Status Register */
    /* LOAD  -->  Offset: 0x004 (R/W)  SysTick Reload Value Register */
    /* VAL   -->  Offset: 0x008 (R/W)  SysTick Current Value Register */
    /* CALIB -->  Offset: 0x00C (R/ )  SysTick Calibration Register */
    /* Systick->LOAD Initial Value -->  SYSTICK_CLOCK_HZ (64MHz) / 1000U ) - 1UL = 64,000 */
#ifndef UNIT_TEST
    uint32_t msTime = GetMilliseconds ();
    if (SysTick->LOAD == SysTick->VAL) {
        return msTime * 1000U;
    }
    uint32_t usTime = (SysTick->LOAD - SysTick->VAL) / (SysTick->LOAD / 1000U);
    return msTime * 1000U + usTime;
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
