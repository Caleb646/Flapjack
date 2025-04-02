#ifndef COMMON_H
#define COMMON_H

#include <stddef.h>
#include <assert.h>

#define STATIC_ASSERT(expr, msg) static_assert((expr), msg)

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

#endif // COMMON_H
