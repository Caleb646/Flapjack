#ifndef COMMON_H
#define COMMON_H

#include "hal.h"
#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

// Test visibility macros
#ifdef UNIT_TEST
#define STATIC_TESTABLE_DECL
#else
#define STATIC_TESTABLE_DECL static
#endif

#define MICRO_DELAY_USE_SYSTICK  0

#define STATIC_ASSERT(expr, msg) static_assert ((expr), msg)
#define ASSERT(expr)             assert ((expr))
#define MASSERT(expr, __VARGS__) assert ((expr))

#define MIN_I32(x, y)            (((x) < (y)) ? (x) : (y))
#define MIN_U32(x, y)            (((x) < (y)) ? (x) : (y))
#define MIN_F32(x, y)            (((x) < (y)) ? (x) : (y))
#define MAX_I32(x, y)            (((x) > (y)) ? (x) : (y))
#define MAX_U32(x, y)            (((x) > (y)) ? (x) : (y))
#define MAX_F32(x, y)            (((x) > (y)) ? (x) : (y))
#define ABS_F32(x)               (((float)(x) < 0.0F) ? -((float)(x)) : (float)(x))

#define DEG2RAD(x)               (((float)(x)) * 0.017453292519943295F) // (π / 180)
#define RAD2DEG(x)               (((float)(x)) * 57.29577951308232F) // (180 / π)

typedef uint8_t BOOL_t;
enum { FALSE = 0, TRUE = 1 };

typedef int8_t eSTATUS_t;
enum {
    eSTATUS_SUCCESS = 0,
    eSTATUS_BUSY    = -1,


    eSTATUS_FAILURE = -126
};

#define STATUS_OK(STATUS)   ((STATUS) == eSTATUS_SUCCESS)
#define STATUS_FAIL(STATUS) ((STATUS) != eSTATUS_SUCCESS)

typedef struct {
    int32_t x, y, z;
} Vec3;

typedef struct {
    union {
        float x;
        float roll;
    };
    union {
        float y;
        float pitch;
    };
    union {
        float z;
        float yaw;
    };
} Vec3f;

typedef struct {
    union {
        float x;
        float roll;
        float q1;
    };
    union {
        float y;
        float pitch;
        float q2;
    };
    union {
        float z;
        float yaw;
        float q3;
    };
    union {
        float w;
        float thrust;
        float q4;
    };
} Vec4f;

void CriticalErrorHandler (void);
inline int32_t clipi32 (int32_t v, int32_t lower, int32_t upper) {
    if (v < lower) {
        return lower;
    }
    if (v > upper) {
        return upper;
    }
    return v;
}

inline float clipf32 (float v, float lower, float upper) {
    if (v < lower) {
        return lower;
    }
    if (v > upper) {
        return upper;
    }
    return v;
}

inline float mapf32 (float v, float fromMin, float fromMax, float toMin, float toMax) {
    if (fromMax == fromMin) {
        return toMin;
    }
    return toMin + ((v - fromMin) / (fromMax - fromMin)) * (toMax - toMin);
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

inline uint32_t GetMilliseconds (void) {
    return pdMS_TO_TICKS (xTaskGetTickCount ());
}

inline uint32_t GetMicroseconds (void) {
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

inline void DelayMicroseconds (uint32_t us) {
    uint32_t usStart = GetMicroseconds ();
    while ((GetMicroseconds () - usStart) < us) {
        __NOP ();
    }
}

inline void fDelayMicroseconds (float us) {
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

/* Source --> Betaflight: https://github.com/betaflight/betaflight/blob/master/src/main/build/atomic.h */
inline void BasePriRestoreMem (uint8_t* val) {
    __set_BASEPRI (*val);
}

// set BASEPRI_MAX, with global memory barrier, returns true
inline uint8_t BasePriSetMemRetVal (uint8_t prio) {
    __set_BASEPRI_MAX (prio);
    return 1;
}

typedef uint8_t eNVIC_PRIO_LVL_t;
enum {
    eNVIC_PRIO_LVL_UNUSED = 0,
    eNVIC_PRIO_LVL_MAX,
    eNVIC_PRIO_LVL_2,
    eNVIC_PRIO_LVL_3,
    eNVIC_PRIO_LVL_4,
    eNVIC_PRIO_LVL_5,
    eNVIC_PRIO_LVL_6,
    eNVIC_PRIO_LVL_7,
    eNVIC_PRIO_LVL_8,
    eNVIC_PRIO_LVL_9,
    eNVIC_PRIO_LVL_10,
    eNVIC_PRIO_LVL_11,
    eNVIC_PRIO_LVL_12,
    eNVIC_PRIO_LVL_13,
    eNVIC_PRIO_LVL_14,
    eNVIC_PRIO_LVL_15
};

/* Source --> Betaflight: https://github.com/betaflight/betaflight/blob/master/src/main/build/atomic.h */
inline void BasePriRestoreMem (uint8_t* val);
// set BASEPRI_MAX, with global memory barrier, returns true
inline uint8_t BasePriSetMemRetVal (uint8_t prio);

// clang-format off
#define ATOMIC_BLOCK_LOCAL(NVIC_PRIO)                                                                    \
    for (uint8_t __basepri_save __attribute__ ((__cleanup__ (BasePriRestoreMem), __unused__)) =  __get_BASEPRI (), __ToDo = BasePriSetMemRetVal ((NVIC_PRIO)); \
    __ToDo; __ToDo = 0); \
    // clang-format on


#endif // COMMON_H
