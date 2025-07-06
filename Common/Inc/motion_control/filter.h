#ifndef MOTION_CONTROL_FILTER_H
#define MOTION_CONTROL_FILTER_H

#include "common.h"
#include "sensors/imu/imu.h"
#include <stdint.h>

// #define DEG2RAD(degrees) ((degrees) * (3.14159265359F / 180.0F))
// #define RAD2DEG(radians) ((radians) * (180.0F / 3.14159265359F))

typedef struct {
    // estimated orientation quaternion elements with initial conditions
    Vec4f est;
    // Gamma_t (γt)
    float beta;
} FilterMadgwickContext;

STATUS_TYPE
FilterMadgwick6DOF (
FilterMadgwickContext* pContext,
Vec3f const* pAccel,
Vec3f const* pGyroDegs,
float dt,
Vec3f* pOutputAttitude);
// STATUS_TYPE
// FilterMadgwick9DOF (FilterMadgwickContext* pContext, Vec3f accel, Vec3f gyro, Vec3f magno, float dt, Vec3f* pOutputAttitude);
// STATUS_TYPE FilterMadgwickInit (FilterMadgwickContext* pContext, float gyroMeasureErrorDegs);

STATUS_TYPE
FilterMadgwickWarmUp (uint32_t iterations, IMU* pIMU, float expectedGyroErrorDegs, float beta, FilterMadgwickContext* pOutContext);

#endif // MOTION_CONTROL_FILTER_H
