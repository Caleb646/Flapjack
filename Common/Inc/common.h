#ifndef COMMON_H
#define COMMON_H

#include <stddef.h>
#include <assert.h>

#define STATIC_ASSERT(expr, msg) static_assert((expr), msg)

typedef struct {
    int32_t x, y, z;
} Vec3;

#endif // COMMON_H
