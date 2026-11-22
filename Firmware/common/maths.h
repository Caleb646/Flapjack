#ifndef COMMON_MATHS_H
#define COMMON_MATHS_H

#include <stdint.h>

#define ABS_F32(x) (((float)(x) < 0.0F) ? -((float)(x)) : (float)(x))

#define DEG2RAD(x) (((float)(x)) * 0.017453292519943295F) // (π / 180)
#define RAD2DEG(x) (((float)(x)) * 57.29577951308232F)    // (180 / π)

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

float clipf32 (float v, float lower, float upper);

#endif // COMMON_MATHS_H
