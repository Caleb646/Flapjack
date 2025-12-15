#ifndef AERO_NAV_H
#define AERO_NAV_H

#include <stdbool.h>
#include <stdint.h>

#include "common.h"

typedef struct Madgwick_s {
    // estimated orientation quaternion elements with initial conditions
    Vec4f qEst;
    // gyro bias error
    Vec3f gbias;
    // Gamma_t (γt)
    float beta;
    float zeta;
    // reference direction of flux in earth frame
    float bx;
    float bz;
} Madgwick_t;

typedef struct Fusion_s {
    Madgwick_t madgwick;
} Fusion_t;

void Madgwick_Init (float gyroMeasureErrorDegs, float gyroMeasureDriftDegs, Madgwick_t* pOutMadgwick);
bool Madgwick_Update_6DOF (Madgwick_t* pMadgwick, Vec3f const* pAccel, Vec3f const* pGyroDegs, float dt);

eSTATUS_t Filter_Init (void);

eSTATUS_t Nav_Init (void);


#endif // AERO_NAV_H