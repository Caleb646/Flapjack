#ifndef CORE_COMMON_H
#define CORE_COMMON_H

#include "hal.h"
#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include "targets/target.h"

#ifdef UNIT_TEST
#define FJ_INLINE
#define FJ_STATIC
#define FJ_UNUSED_FN_DECL
#define FJ_TESTABLE
#else
#define FJ_INLINE         inline
#define FJ_STATIC         static
#define FJ_UNUSED_FN_DECL __attribute__ ((unused))
#define FJ_TESTABLE       FJ_STATIC
#endif

#define FJ_UNUSED(x)                (void)(x)
#define FJ_STATIC_ASSERT(expr, msg) static_assert ((expr), msg)
#define FJ_ASSERT(expr)             assert ((expr))
#define FJ_MASSERT(expr, __VARGS__) assert ((expr))

#define MIN_I32(x, y)               (((x) < (y)) ? (x) : (y))
#define MIN_U32(x, y)               (((x) < (y)) ? (x) : (y))
#define MIN_F32(x, y)               (((x) < (y)) ? (x) : (y))
#define MAX_I32(x, y)               (((x) > (y)) ? (x) : (y))
#define MAX_U32(x, y)               (((x) > (y)) ? (x) : (y))
#define MAX_F32(x, y)               (((x) > (y)) ? (x) : (y))
#define ABS_F32(x)                  (((float)(x) < 0.0F) ? -((float)(x)) : (float)(x))

#define DEG2RAD(x)                  (((float)(x)) * 0.017453292519943295F) // (π / 180)
#define RAD2DEG(x)                  (((float)(x)) * 57.29577951308232F)    // (180 / π)

#define FJ_CTZ(X)                   ((X) == 0 ? 0U : __builtin_ctz (X))
#define BUF_TO_U16(BUF)             ((uint16_t)(((uint32_t)(BUF)[1] << 8U) | (uint16_t)(BUF)[0]))

#define ARRAY_SIZE(ARRAY)           (sizeof (ARRAY) / sizeof ((ARRAY)[0]))

#define FOR_EACH(TYPE, ARRAY) \
    for (TYPE* pElement = &((ARRAY)[0]); pElement <= &((ARRAY)[ARRAY_SIZE (ARRAY) - 1U]); ++pElement)
#define FOR_EACH_CONST(TYPE, ARRAY)                   FOR_EACH (TYPE const, ARRAY)

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

typedef uint16_t eSTATUS_t;
enum {
    eSTATUS_OK   = (1U << 0U),
    eSTATUS_FAIL = (1U << 1U),
    eSTATUS_BUSY = (1U << 2U)


};

// clang-format off

#define STATUS_OK(STATUS)   ((STATUS) == eSTATUS_OK)
#define STATUS_OK_BUSY(STATUS) ((STATUS) == eSTATUS_OK || (STATUS) == eSTATUS_BUSY)
#define STATUS_OK_PREV_INITED(STATUS) ((STATUS) == eSTATUS_OK || (STATUS) == eSTATUS_ALREADY_INITED)
#define STATUS_FAIL(STATUS) ((STATUS) != eSTATUS_OK)
#define STATUS_RETRY(STATUS) ((STATUS) == eSTATUS_RETRY)
#define FJ_OK(STATUS)    STATUS_OK(STATUS)
#define FJ_FAIL(STATUS)    STATUS_FAIL(STATUS)
#define FJ_IS_NULL(POINTER) ((POINTER) == NULL)

#define IS_CM7_ME()     (HAL_GetCurrentCPUID() == CM7_CPUID)
#define IS_CM4_ME()     (HAL_GetCurrentCPUID() == CM4_CPUID)

// clang-format on

typedef uint8_t AXIS_IDX_t;
enum {
    AXIS_IDX_ROLL = 0,
    AXIS_IDX_PITCH,
    AXIS_IDX_YAW,
    AXIS_IDX_COUNT,
};

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

#define VEC3F_ZERO() { .x = 0.0F, .y = 0.0F, .z = 0.0F }

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

#define GET_FIRST_EXPAND(FIRST, ...)                FIRST
#define GET_FIRST(...)                              GET_FIRST_EXPAND (__VA_ARGS__)
#define GET_SECOND_EXPAND(FIRST, SECOND, ...)       SECOND
#define GET_SECOND(...)                             GET_SECOND_EXPAND (__VA_ARGS__)
#define GET_THIRD_EXPAND(FIRST, SECOND, THIRD, ...) THIRD
#define GET_THIRD(...)                              GET_THIRD_EXPAND (__VA_ARGS__)

#define VEC4F_ZERO() { .x = 0.0F, .y = 0.0F, .z = 0.0F, .w = 0.0F }
// clang-format off
#define MEM_U32_ALIGN4(addr)                  ((uint32_t)(addr) & ((uint32_t)~0x3U))

#define FJ_DECLARE_SHARED(TYPE, NAME)      extern TYPE NAME TARG_SHARED_MEM_DATA_SECTION
#define FJ_DEFINE_SHARED(TYPE, NAME)       TYPE NAME TARG_SHARED_MEM_DATA_SECTION

// clang-format on
void* Alloc_SharedMem (size_t size);

void CriticalErrorHandler (void);

int32_t clipi32 (int32_t v, int32_t lower, int32_t upper);
float clipf32 (float v, float lower, float upper);
float mapf32 (float v, float fromMin, float fromMax, float toMin, float toMax);


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

#define US_TO_SECONDS(US)              ((float)(US) / 1000000.0F)
uint32_t GetMilliseconds (void);
uint32_t GetMicroseconds (void);
void Delay (uint32_t ms);
void DelayMicroseconds (uint32_t us);


#endif // CORE_COMMON_H
