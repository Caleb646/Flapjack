#ifndef FILTER_H
#define FILTER_H

#include <stdint.h>
#include "common.h"

typedef struct FilterMadgwickContext__ {
    // estimated orientation quaternion elements with initial conditions
    float seq1, seq2, seq3, seq4;
    // sampling period in seconds
    float dt; 
    // gyroscope measurement error in rad/s
    float gyroMeasureError; 
    float beta;
} FilterMadgwickContext;

int8_t FilterMadgwick(FilterMadgwickContext *pContext, Vec3 gyro, Vec3 accel);
int8_t FilterMadgwickwMagnetometer(FilterMadgwickContext *pContext, Vec3 gyro, Vec3 accel, Vec3 magno);
int8_t FilterMadgwickInit(FilterMadgwickContext *pContext);

#endif // FILTER_H
