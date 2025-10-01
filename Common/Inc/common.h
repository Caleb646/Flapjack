#ifndef COMMON_H
#define COMMON_H

#include "hal.h"
#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

// Test visibility macros
#ifdef UNIT_TEST
#define STATIC_TESTABLE_DECL
#define INLINE
#define UNUSED_FN_DECL
#else
#define INLINE               inline
#define STATIC_TESTABLE_DECL static
#define UNUSED_FN_DECL       __attribute__ ((unused))
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

#define FOR_EACH(pARRAY, SIZE, FN, ...)         \
    do {                                        \
        for (uint32_t i = 0; i < (SIZE); ++i) { \
            FN ((pARRAY) + i, ##__VA_ARGS__);   \
        }                                       \
    } while (0)

typedef int8_t eSTATUS_t;
enum {

    eSTATUS_FAILURE = -126,
    eSTATUS_BUSY    = -124,
    eSTATUS_TIMEOUT = -123,

    eSTATUS_SUB_STATUS_START,

    eSTATUS_SUCCESS = 0,
};

#define STATUS_OK(STATUS)   ((STATUS) == eSTATUS_SUCCESS)
#define STATUS_FAIL(STATUS) ((STATUS) != eSTATUS_SUCCESS)

typedef struct {
    int32_t x, y, z;
} Vec3i;

typedef struct {
    uint32_t x, y, z;
} Vec3u;

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
int32_t clipi32 (int32_t v, int32_t lower, int32_t upper);
float clipf32 (float v, float lower, float upper);
float mapf32 (float v, float fromMin, float fromMax, float toMin, float toMax);
uint32_t GetMilliseconds (void);
uint32_t GetMicroseconds (void);
void DelayMicroseconds (uint32_t us);
void fDelayMicroseconds (float us);
/* Source --> Betaflight: https://github.com/betaflight/betaflight/blob/master/src/main/build/atomic.h */
void BasePriRestoreMem (uint8_t* val);
// set BASEPRI_MAX, with global memory barrier, returns true
uint8_t BasePriSetMemRetVal (uint8_t prio);

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

// clang-format off
// NOTE: Be careful using this with Delay functions as 
// they may use SysTick which may be disabled by this macro.
#define ATOMIC_BLOCK_LOCAL(NVIC_PRIO)                                                                    \
    for (uint8_t __basepri_save __attribute__ ((__cleanup__ (BasePriRestoreMem), __unused__)) =  __get_BASEPRI (), __ToDo = BasePriSetMemRetVal ((NVIC_PRIO)); \
    __ToDo; __ToDo = 0); \
    // clang-format on


#endif // COMMON_H
