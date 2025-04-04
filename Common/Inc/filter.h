#ifndef FILTER_H
#define FILTER_H

#include <stdint.h>
#include "common.h"

typedef struct {
    // estimated orientation quaternion elements with initial conditions
    float seq1, seq2, seq3, seq4;
    // sampling period in seconds
    float dt; 
    // gyroscope measurement error in rad/s
    float gyroMeasureError; 
    float beta;
} FilterMadgwickContext;

int8_t FilterMadgwick6DOF(FilterMadgwickContext *pContext, Vec3f gyro, Vec3f accel, Vec3f *pOutputAttitude);
int8_t FilterMadgwick9DOF(FilterMadgwickContext *pContext, Vec3f gyro, Vec3f accel, Vec3f magno, Vec3f *pOutputAttitude);
int8_t FilterMadgwickInit(FilterMadgwickContext *pContext);

#endif // FILTER_H
