#ifndef CORE_COMMON_H
#define CORE_COMMON_H

#include "hal.h"
#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#ifdef UNIT_TEST
#define INLINE
#define STATIC
#define UNUSED_FN_DECL
#else
#define INLINE         inline
#define STATIC         static
#define UNUSED_FN_DECL __attribute__ ((unused))
#endif

#define MICRO_DELAY_USE_SYSTICK  0

#define FJ_UNUSED(x)             (void)(x)
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
#define RAD2DEG(x)               (((float)(x)) * 57.29577951308232F)    // (180 / π)

#define BUF_TO_U16(BUF)          ((uint16_t)(((uint32_t)(BUF)[1] << 8U) | (uint16_t)(BUF)[0]))

#define ARRAY_SIZE(ARRAY)        (sizeof (ARRAY) / sizeof ((ARRAY)[0]))

#define FOR_EACH(TYPE, ARRAY) \
    for (TYPE* pElement = &((ARRAY)[0]); pElement <= &((ARRAY)[ARRAY_SIZE (ARRAY) - 1U]); ++pElement)
#define FOR_EACH_CONST(TYPE, ARRAY)                   FOR_EACH (TYPE const, ARRAY)

#define AXIS_IDX_COUNT                                4
#define AXIS_IDX_ROLL                                 0
#define AXIS_IDX_PITCH                                1
#define AXIS_IDX_YAW                                  2
#define AXIS_IDX_THROTTLE                             3

#define SHARED_MEM_BSS_SECTION                        __attribute__ ((section (".shared_mem_bss")))
#define SHARED_MEM_DATA_SECTION                       __attribute__ ((section (".shared_mem_data")))
#define FJ_DECLARE_SHARED(TYPE, NAME)                 extern TYPE NAME SHARED_MEM_DATA_SECTION
#define FJ_DEFINE_SHARED(TYPE, NAME)                  TYPE NAME SHARED_MEM_DATA_SECTION

#define CM7_IDX                                       0
#define CM4_IDX                                       1


#define OR_0()                                        0U
#define OR_1(a)                                       (a)
#define OR_2(a, b)                                    ((a) | (b))
#define OR_3(a, b, c)                                 ((a) | (b) | (c))
#define OR_4(a, b, c, d)                              ((a) | (b) | (c) | (d))
#define OR_5(a, b, c, d, e)                           ((a) | (b) | (c) | (d) | (e))
#define OR_6(a, b, c, d, e, f)                        ((a) | (b) | (c) | (d) | (e) | (f))
#define OR_7(a, b, c, d, e, f, g)                     ((a) | (b) | (c) | (d) | (e) | (f) | (g))
#define OR_8(a, b, c, d, e, f, g, h)                  ((a) | (b) | (c) | (d) | (e) | (f) | (g) | (h))
#define NARG_(a1, a2, a3, a4, a5, a6, a7, a8, N, ...) N
#define NARG(...)                                     NARG_ (__VA_ARGS__, 8, 7, 6, 5, 4, 3, 2, 1, 0)
#define CONCAT_(a, b)                                 a##b
#define CONCAT(a, b)                                  CONCAT_ (a, b)
#define VALUES(NAME, ...)                             CONCAT (NAME, NARG (__VA_ARGS__)) (__VA_ARGS__)

#define HZ_TO_US(HZ)                                  ((HZ) > 0 ? (1000000U / (HZ)) : 0U)

typedef int8_t eSTATUS_t;
enum {
    eSTATUS_FAILURE        = -126,
    eSTATUS_BUSY           = -124,
    eSTATUS_TIMEOUT        = -123,
    eSTATUS_NULL_ARG       = -122,
    eSTATUS_INVALID_ARG    = -121,
    eSTATUS_BUS_ERROR      = -120,
    eSTATUS_MEM_ERROR      = -119,
    eSTATUS_HW_ERROR       = -118,
    eSTATUS_DEV_ERROR      = -117,
    eSTATUS_EXT_DEV_ERROR  = -115,
    eSTATUS_NOT_FOUND      = -116,
    eSTATUS_UNSUPPORTED    = -115,
    eSTATUS_ALREADY_INITED = -114,
    eSTATUS_RETRY          = -113,

    eSTATUS_SUB_STATUS_START__,

    eSTATUS_SUCCESS = 0,
};

// clang-format off

#define STATUS_OK(STATUS)   ((STATUS) == eSTATUS_SUCCESS)
#define STATUS_OK_BUSY(STATUS) ((STATUS) == eSTATUS_SUCCESS || (STATUS) == eSTATUS_BUSY)
#define STATUS_OK_PREV_INITED(STATUS) ((STATUS) == eSTATUS_SUCCESS || (STATUS) == eSTATUS_ALREADY_INITED)
#define STATUS_FAIL(STATUS) ((STATUS) != eSTATUS_SUCCESS)
#define STATUS_RETRY(STATUS) ((STATUS) == eSTATUS_RETRY)

// clang-format on

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

extern float ge_ScaledSystemCoreClock;

#define CURRENT_CYCLES() (DWT->CYCCNT)
#define US_TO_CYCLES(US_FLOAT) \
    ((uint32_t)((((US_FLOAT) * 10.0F) * ge_ScaledSystemCoreClock) + 0.5F))

#define DELAY_CYCLES(CYCLES_UINT32)                             \
    {                                                           \
        uint32_t start = CURRENT_CYCLES ();                     \
        while ((CURRENT_CYCLES () - start) < (CYCLES_UINT32)) { \
            __NOP ();                                           \
        }                                                       \
    }
#define F_DELAY_MICROSECONDS(US_FLOAT) DELAY_CYCLES (US_TO_CYCLES (US_FLOAT))


eSTATUS_t CoreShared_Init (void);

void CriticalErrorHandler (void);
int32_t clipi32 (int32_t v, int32_t lower, int32_t upper);
float clipf32 (float v, float lower, float upper);
float mapf32 (float v, float fromMin, float fromMax, float toMin, float toMax);
uint32_t GetMilliseconds (void);
uint32_t GetMicroseconds (void);
void Delay (uint32_t ms);
void DelayMicroseconds (uint32_t us);
// void fDelayMicroseconds (float us);
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

#ifndef UNIT_TEST

// clang-format off
// NOTE: Be careful using this with Delay functions as 
// they may use SysTick which may be disabled by this macro.
#define ATOMIC_BLOCK_LOCAL(NVIC_PRIO)                                                                    \
    for (uint8_t __basepri_save __attribute__ ((__cleanup__ (BasePriRestoreMem), __unused__)) =  __get_BASEPRI (), __ToDo = BasePriSetMemRetVal ((NVIC_PRIO)); \
    __ToDo; __ToDo = 0); \
    // clang-format on

#else

#define ATOMIC_BLOCK_LOCAL(NVIC_PRIO) if (1)

#endif


#endif // CORE_COMMON_H
