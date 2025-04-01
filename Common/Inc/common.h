#ifndef COMMON_H
#define COMMON_H

#include <stddef.h>
#include <assert.h>

#define STATIC_ASSERT(expr, msg) static_assert((expr), msg)

typedef struct {
    uint32_t x, y, z;
} UVec3;

#endif // COMMON_H
