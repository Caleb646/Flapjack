#ifndef COMMON_H
#define COMMON_H

#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>


// Test visibility macros
#ifdef UNIT_TEST
// #define STATIC_TESTABLE
// extern
#define STATIC_TESTABLE_DECL
#else
// #define STATIC_TESTABLE      static
#define STATIC_TESTABLE_DECL static
#endif

#define STATIC_ASSERT(expr, msg) static_assert ((expr), msg)
#define ASSERT(expr)             assert ((expr))
#define MASSERT(expr, __VARGS__) assert ((expr))


// ASSERT_LOG macro: logs message, location info, and calls CriticalErrorHandler on failure
// This version uses the LOG_ERROR macro from log.h
// #define ASSERT_LOG(condition, message, ...) \
//     do { \
//         if (!(condition)) { \
//             LOG_ERROR ("ASSERTION FAILED: %s", #condition); \
//             LOG_ERROR ("Message: " message, ##__VA_ARGS__); \
//             LOG_ERROR ("Location: %s() in %s:%d", __func__, __FILENAME__, __LINE__); \
//             PrintStackTrace (); \
//             CriticalErrorHandler (); \
//         } \
//     } while (0)

// #define ASSERT_LOG_NO_PRINTF(condition, message, ...)                    \
//     do {                                                                 \
//         if (!(condition)) {                                              \
//             /* Store assertion info in global variables for debugging */ \
//             AssertionInfo.file      = __FILENAME__;                      \
//             AssertionInfo.line      = __LINE__;                          \
//             AssertionInfo.function  = __func__;                          \
//             AssertionInfo.condition = #condition;                        \
//             CriticalErrorHandler ();                                     \
//         }                                                                \
//     } while (0)
// Structure to hold assertion information when printf is not available
// typedef struct {
//     const char* file;
//     int line;
//     const char* function;
//     const char* condition;
// } AssertionInfo_t;

// extern AssertionInfo_t AssertionInfo;


#define MIN_I32(x, y)            ((x < y) ? x : y)
#define MIN_U32(x, y)            ((x < y) ? x : y)
#define MIN_F32(x, y)            ((x < y) ? x : y)
#define MAX_I32(x, y)            ((x > y) ? x : y)
#define MAX_U32(x, y)            ((x > y) ? x : y)
#define MAX_F32(x, y)            ((x > y) ? x : y)

#define DEG2RAD(x)               (((float)x) * 0.017453292519943295F) // (π / 180)
#define RAD2DEG(x)               (((float)x) * 57.29577951308232F) // (180 / π)

#define TRUE                     (1)
#define FALSE                    (0)

typedef enum {
    eSTATUS_SUCCESS = 0,
    eSTATUS_BUSY    = -1,


    eSTATUS_FAILURE = -127
} STATUS_TYPE;

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
