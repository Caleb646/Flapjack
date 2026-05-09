#ifndef MOTION_CONTROL_FILTER_H
#define MOTION_CONTROL_FILTER_H

#include "core/core.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    struct {
        float alpha;
        bool isFirstUpdate;
    } cfg;
    uint32_t usLastUpdateTime;
} LowPassFilter_t;

typedef struct {
    // estimated orientation quaternion elements with initial conditions
    Vec4f qEst;
    // gyro bias error
    Vec3f gbias;
    struct {
        float gyroMeasureErrorDegs;
        float gyroMeasureDriftDegs;
    } cfg;
    // Gamma_t (γt)
    float beta;
    float zeta;
    // reference direction of flux in earth frame
    float bx;
    float bz;
    uint32_t usLastUpdateTime;
} MadgwickFilter_t;

// clang-format off

void MadgwickFilter_Update_6DOF_ (MadgwickFilter_t* pFilter, Vec3f const* pAccel, Vec3f const* pGyroDegs, float dt);
void MadgwickFilter_Update_9DOF_ (MadgwickFilter_t* pFilter, Vec3f const* pAccel, Vec3f const* pGyroDegs, Vec3f const* pMag, float dt);
eSTATUS_t MadgwickFilter_Update (MadgwickFilter_t* pFilter, Vec3f const* pAccel, Vec3f const* pGyro, Vec3f const* pMag, float dt, Vec3f* pOutAttitude);
void MadgwickFilter_QuatToEuler (MadgwickFilter_t const* pFilter, Vec3f* pOutEuler);
eSTATUS_t MadgwickFilter_Init (MadgwickFilter_t* pFilter);

eSTATUS_t LowPassFilter_Init (LowPassFilter_t* pFilter);

// clang-format on

#endif // MOTION_CONTROL_FILTER_H
