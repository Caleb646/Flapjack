#ifndef COMMON_H
#define COMMON_H

#include <stdint.h>
#include <stddef.h>
#include <assert.h>

#define STATIC_ASSERT(expr, msg) static_assert((expr), msg)
#define MIN_I32(x, y) ((x < y) ? x : y)
#define MIN_U32(x, y) ((x < y) ? x : y)
#define MIN_F32(x, y) ((x < y) ? x : y)
#define MAX_I32(x, y) ((x > y) ? x : y)
#define MAX_U32(x, y) ((x > y) ? x : y)
#define MAX_F32(x, y) ((x > y) ? x : y)

typedef struct {
    int32_t x, y, z;
} Vec3;

typedef struct {
    union
    {
        float x;
        float pitch;
    };
    union
    {
        float y;
        float roll;
    };
    union
    {
        float z;
        float yaw;
    };
} Vec3f;

int32_t clipi32(int32_t v, int32_t lower, int32_t upper);
float clipf32(float v, float lower, float upper);

#endif // COMMON_H
