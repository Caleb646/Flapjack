#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "common.h"
#include "hal.h"

#include "targets/target.h"

#define SHARED_MEM_SIZE 2048U
DEFINE_STATIC_SHARED_BSS_ARRAY (uint8_t, g_SharedMem, SHARED_MEM_SIZE);
DEFINE_STATIC_SHARED_BSS (uint8_t*, gp_CurSharedMem);

void* Alloc_SharedMem (size_t size) {

    if (!gp_CurSharedMem) {
        gp_CurSharedMem = g_SharedMem;
    }

    uintptr_t mem = MEM_U32_ALIGN4 (((uintptr_t)gp_CurSharedMem) + 3U);
    if ((mem + size) > SHARED_MEM_SIZE) {
        return NULL;
    }

    gp_CurSharedMem = (uint8_t*)(mem + size);
    return (void*)mem;
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
    // if (fromMax == fromMin) {
    //     return toMin;
    // }
    return toMin + ((v - fromMin) / ((fromMax - fromMin) + 0.0001F)) * (toMax - toMin);
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
    // #if MICRO_DELAY_USE_SYSTICK == 1
    //     if (SysTick->LOAD == SysTick->VAL) {
    //         return msTime * 1000U;
    //     }
    //     uint32_t usTime = (SysTick->LOAD - SysTick->VAL) / (SysTick->LOAD / 1000U);
    //     return msTime * 1000U + usTime;
    // #else
    uint32_t usTime = DWT->CYCCNT / (SystemCoreClock / 1000000U);
    return msTime * 1000U + usTime;
// #endif
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
