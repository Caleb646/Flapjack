#ifndef COMMON_H
#define COMMON_H

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

#define STATIC_ASSERT(expr, msg) static_assert ((expr), msg)
#define ASSERT(expr)             assert ((expr))
#define MASSERT(expr, __VARGS__) assert ((expr))

#define MIN_I32(x, y)            ((x < y) ? x : y)
#define MIN_U32(x, y)            ((x < y) ? x : y)
#define MIN_F32(x, y)            ((x < y) ? x : y)
#define MAX_I32(x, y)            ((x > y) ? x : y)
#define MAX_U32(x, y)            ((x > y) ? x : y)
#define MAX_F32(x, y)            ((x > y) ? x : y)

#define DEG2RAD(x)               (((float)x) * 0.017453292519943295F) // (π / 180)
#define RAD2DEG(x)               (((float)x) * 57.29577951308232F) // (180 / π)

typedef uint8_t BOOL_t;
enum { FALSE = 0, TRUE = 1 };

typedef int8_t eSTATUS_t;
enum {
    eSTATUS_SUCCESS = 0,
    eSTATUS_BUSY    = -1,


    eSTATUS_FAILURE = -126
};

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
int32_t clipi32 (int32_t v, int32_t lower, int32_t upper);
float clipf32 (float v, float lower, float upper);
float mapf32 (float v, float fromMin, float fromMax, float toMin, float toMax);
void DelayMicroseconds (uint32_t us);

#endif // COMMON_H
