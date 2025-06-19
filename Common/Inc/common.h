#ifndef COMMON_H
#define COMMON_H

#include <assert.h>
#include <stddef.h>
#include <stdint.h>

#define STATIC_ASSERT(expr, msg) static_assert ((expr), msg)
#define ASSERT(expr)             assert ((expr))
#define MASSERT(expr, __VARGS__) assert ((expr))

#define MIN_I32(x, y)            ((x < y) ? x : y)
#define MIN_U32(x, y)            ((x < y) ? x : y)
#define MIN_F32(x, y)            ((x < y) ? x : y)
#define MAX_I32(x, y)            ((x > y) ? x : y)
#define MAX_U32(x, y)            ((x > y) ? x : y)
#define MAX_F32(x, y)            ((x > y) ? x : y)

typedef enum {
    eSTATUS_SUCCESS = 0,


    eSTATUS_FAILURE = -127
} STATUS_TYPE;

// #define IS_STATUS_SUCCESS(status) (status == eSTATUS_SUCCESS)
// #define IS_STATUS_FAILURE(status) (status != eSTATUS_SUCCESS)

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

void CriticalErrorHandler (void);
int32_t clipi32 (int32_t v, int32_t lower, int32_t upper);
float clipf32 (float v, float lower, float upper);
float mapf32 (float v, float fromMin, float fromMax, float toMin, float toMax);

#endif // COMMON_H
