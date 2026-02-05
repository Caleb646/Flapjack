#include "mc/filter.h"
#include "core/core.h"
#include "core/log/logger.h"
#include "device/imu/imu.h"
#include "device/mag/mag.h"
#include "hal.h"
#include "mem/mem.h"
#include <math.h>
#include <stdbool.h>
#include <string.h>


#define FILTER_VALID(pF) ((pF) != NULL && (pF)->isInitialized == true)

static SHARED_MEM_SECTION Filter_t gFilter = { 0 };

// clang-format off
#ifndef UNIT_TEST

static bool FilterMadgwickUpdate (FilterMadgwick_t* pFilter,Vec3f const* pAccel,Vec3f const* pGyro,Vec3f const* pMag,float dt,Vec3f* pOutAttitude);
static bool FilterMadgwickUpdate_6DOF (FilterMadgwick_t* pFilter, Vec3f const* pAccel, Vec3f const* pGyroDegs, float dt);
static bool FilterMadgwickUpdate_9DOF (FilterMadgwick_t* pFilter,Vec3f const* pAccel,Vec3f const* pGyro,Vec3f const* pMag,float dt);
static bool FilterMadgwickInit (FilterMadgwickInitConf_t conf, FilterMadgwick_t* pOut);

#endif
// clang-format on

/*
 * The filter expects the accel and gyro data to be in the FRD coordinate
 * system. For example, an accel reading of (0, 0, +1g) means the sensor is
 * stationary and level. So the returned attitude will move towards (0, 0,
 * 0). If the IMU is stationary and level but has an accel reading of (0,
 * 1g, 0) then the returned attitude will move towards (0, -90, 0).
 * Which is incorrect so the IMU data needs to be in FRD frame.
 */
STATIC
bool FilterMadgwickUpdate_6DOF (FilterMadgwick_t* pFilter, Vec3f const* pAccel, Vec3f const* pGyroDegs, float dt) {
    // Source: https://courses.cs.washington.edu/courses/cse474/17wi/labs/l4/madgwick_internal_report.pdf

    // convert degrees per second to radians per second
    float a_x  = pAccel->x;
    float a_y  = pAccel->y;
    float a_z  = pAccel->z;
    float w_x  = DEG2RAD (pGyroDegs->x);
    float w_y  = DEG2RAD (pGyroDegs->y);
    float w_z  = DEG2RAD (pGyroDegs->z);
    float beta = pFilter->beta;

    float SEq_1 = pFilter->qEst.q1;
    float SEq_2 = pFilter->qEst.q2;
    float SEq_3 = pFilter->qEst.q3;
    float SEq_4 = pFilter->qEst.q4;

    float norm           = 0.0F;
    float SEqDot_omega_1 = 0.0F;
    float SEqDot_omega_2 = 0.0F;
    float SEqDot_omega_3 = 0.0F;
    float SEqDot_omega_4 = 0.0F; // quaternion derrivative from gyroscopes elements float
    float f_1            = 0.0F;
    float f_2            = 0.0F;
    float f_3            = 0.0F; // objective function elements
    float J_11or24       = 0.0F;
    float J_12or23       = 0.0F;
    float J_13or22       = 0.0F;
    float J_14or21       = 0.0F;
    float J_32           = 0.0F;
    float J_33           = 0.0F;
    float SEqHatDot_1    = 0.0F;
    float SEqHatDot_2    = 0.0F;
    float SEqHatDot_3    = 0.0F;
    float SEqHatDot_4    = 0.0F; // Axulirary variables to avoid reapeated calcualtions
    float halfSEq_1      = 0.5F * SEq_1;
    float halfSEq_2      = 0.5F * SEq_2;
    float halfSEq_3      = 0.5F * SEq_3;
    float halfSEq_4      = 0.5F * SEq_4;
    float twoSEq_1       = 2.0F * SEq_1;
    float twoSEq_2       = 2.0F * SEq_2;
    float twoSEq_3       = 2.0F * SEq_3;

    norm = sqrtf (a_x * a_x + a_y * a_y + a_z * a_z);
    if (norm == 0.0F) {
        return false;
    }
    norm = 1.0F / norm;
    a_x *= norm;
    a_y *= norm;
    a_z *= norm;

    // Compute the objective function and Jacobian
    f_1 = twoSEq_2 * SEq_4 - twoSEq_1 * SEq_3 - a_x;
    f_2 = twoSEq_1 * SEq_2 + twoSEq_3 * SEq_4 - a_y;
    f_3 = 1.0F - twoSEq_2 * SEq_2 - twoSEq_3 * SEq_3 - a_z;

    J_11or24 = twoSEq_3;
    J_12or23 = 2.0F * SEq_4;
    J_13or22 = twoSEq_1;
    J_14or21 = twoSEq_2;
    J_32     = 2.0F * J_14or21;
    J_33     = 2.0F * J_11or24;

    SEqHatDot_1 = J_14or21 * f_2 - J_11or24 * f_1;
    SEqHatDot_2 = J_12or23 * f_1 + J_13or22 * f_2 - J_32 * f_3;
    SEqHatDot_3 = J_12or23 * f_2 - J_33 * f_3 - J_13or22 * f_1;
    SEqHatDot_4 = J_14or21 * f_1 + J_11or24 * f_2;

    float factor =
    sqrtf (SEqHatDot_1 * SEqHatDot_1 + SEqHatDot_2 * SEqHatDot_2 + SEqHatDot_3 * SEqHatDot_3 + SEqHatDot_4 * SEqHatDot_4);
    if (factor > 0.0F) {
        norm = 1.0F / factor;
        SEqHatDot_1 *= norm;
        SEqHatDot_2 *= norm;
        SEqHatDot_3 *= norm;
        SEqHatDot_4 *= norm;
    }


    SEqDot_omega_1 = -halfSEq_2 * w_x - halfSEq_3 * w_y - halfSEq_4 * w_z;
    SEqDot_omega_2 = halfSEq_1 * w_x + halfSEq_3 * w_z - halfSEq_4 * w_y;
    SEqDot_omega_3 = halfSEq_1 * w_y - halfSEq_2 * w_z + halfSEq_4 * w_x;
    SEqDot_omega_4 = halfSEq_1 * w_z + halfSEq_2 * w_y - halfSEq_3 * w_x;

    SEq_1 += (SEqDot_omega_1 - (beta * SEqHatDot_1)) * dt;
    SEq_2 += (SEqDot_omega_2 - (beta * SEqHatDot_2)) * dt;
    SEq_3 += (SEqDot_omega_3 - (beta * SEqHatDot_3)) * dt;
    SEq_4 += (SEqDot_omega_4 - (beta * SEqHatDot_4)) * dt;

    norm = 1.0F / sqrtf (SEq_1 * SEq_1 + SEq_2 * SEq_2 + SEq_3 * SEq_3 + SEq_4 * SEq_4);
    SEq_1 *= norm;
    SEq_2 *= norm;
    SEq_3 *= norm;
    SEq_4 *= norm;

    pFilter->qEst.q1 = SEq_1;
    pFilter->qEst.q2 = SEq_2;
    pFilter->qEst.q3 = SEq_3;
    pFilter->qEst.q4 = SEq_4;

    return true;
}

STATIC bool
FilterMadgwickUpdate_9DOF (FilterMadgwick_t* pFilter, Vec3f const* pAccel, Vec3f const* pGyro, Vec3f const* pMag, float dt) {

    float SEq_1 = pFilter->qEst.q1;
    float SEq_2 = pFilter->qEst.q2;
    float SEq_3 = pFilter->qEst.q3;
    float SEq_4 = pFilter->qEst.q4;
    float b_x   = pFilter->bx;
    float b_z   = pFilter->bz;
    float beta  = pFilter->beta;
    float zeta  = pFilter->zeta;
    float w_bx  = pFilter->gbias.x;
    float w_by  = pFilter->gbias.y;
    float w_bz  = pFilter->gbias.z;
    float a_x   = pAccel->x;
    float a_y   = pAccel->y;
    float a_z   = pAccel->z;
    float w_x   = DEG2RAD (pGyro->x);
    float w_y   = DEG2RAD (pGyro->y);
    float w_z   = DEG2RAD (pGyro->z);
    float m_x   = pMag->x;
    float m_y   = pMag->y;
    float m_z   = pMag->z;

    float norm = 0.0F;
    // quaternion rate from gyroscopes elements
    float SEqDot_omega_1 = 0.0F, SEqDot_omega_2 = 0.0F, SEqDot_omega_3 = 0.0F, SEqDot_omega_4 = 0.0F;
    // objective function elements
    float f_1 = 0.0F, f_2 = 0.0F, f_3 = 0.0F, f_4 = 0.0F, f_5 = 0.0F, f_6 = 0.0F;
    // objective function Jacobian elements
    float J_11or24 = 0.0F, J_12or23 = 0.0F, J_13or22 = 0.0F, J_14or21 = 0.0F, J_32 = 0.0F,
          J_33 = 0.0F, J_41 = 0.0F, J_42 = 0.0F, J_43 = 0.0F, J_44 = 0.0F, J_51 = 0.0F, J_52 = 0.0F,
          J_53 = 0.0F, J_54 = 0.0F, J_61 = 0.0F, J_62 = 0.0F, J_63 = 0.0F, J_64 = 0.0F;
    // estimated direction of the gyroscope error
    float SEqHatDot_1 = 0.0F, SEqHatDot_2 = 0.0F, SEqHatDot_3 = 0.0F, SEqHatDot_4 = 0.0F;
    // estimated direction of the gyroscope error (angular)
    float w_err_x = 0.0F, w_err_y = 0.0F, w_err_z = 0.0F;
    // computed flux in the earth frame
    float h_x = 0.0F, h_y = 0.0F, h_z = 0.0F;
    // auxiliary variables to avoid repeated calculations
    float halfSEq_1   = 0.5F * SEq_1;
    float halfSEq_2   = 0.5F * SEq_2;
    float halfSEq_3   = 0.5F * SEq_3;
    float halfSEq_4   = 0.5F * SEq_4;
    float twoSEq_1    = 2.0F * SEq_1;
    float twoSEq_2    = 2.0F * SEq_2;
    float twoSEq_3    = 2.0F * SEq_3;
    float twoSEq_4    = 2.0F * SEq_4;
    float twob_x      = 2.0F * b_x;
    float twob_z      = 2.0F * b_z;
    float twob_xSEq_1 = 2.0F * b_x * SEq_1;
    float twob_xSEq_2 = 2.0F * b_x * SEq_2;
    float twob_xSEq_3 = 2.0F * b_x * SEq_3;
    float twob_xSEq_4 = 2.0F * b_x * SEq_4;
    float twob_zSEq_1 = 2.0F * b_z * SEq_1;
    float twob_zSEq_2 = 2.0F * b_z * SEq_2;
    float twob_zSEq_3 = 2.0F * b_z * SEq_3;
    float twob_zSEq_4 = 2.0F * b_z * SEq_4;
    float SEq_1SEq_2  = 0.0F;
    float SEq_1SEq_3  = SEq_1 * SEq_3;
    float SEq_1SEq_4  = 0.0F;
    float SEq_2SEq_3  = 0.0F;
    float SEq_2SEq_4  = SEq_2 * SEq_4;
    float SEq_3SEq_4  = 0.0F;
    float twom_x      = 2.0F * m_x;
    float twom_y      = 2.0F * m_y;
    float twom_z      = 2.0F * m_z;
    // normalise the accelerometer measurement
    norm = sqrtf (a_x * a_x + a_y * a_y + a_z * a_z);
    if (norm == 0.0F) {
        return false;
    }
    norm = 1.0F / norm;
    a_x *= norm;
    a_y *= norm;
    a_z *= norm;
    // normalise the magnetometer measurement
    norm = sqrtf (m_x * m_x + m_y * m_y + m_z * m_z);
    if (norm == 0.0F) {
        return false;
    }
    norm = 1.0F / norm;
    m_x *= norm;
    m_y *= norm;
    m_z *= norm;
    // compute the objective function and Jacobian
    f_1 = twoSEq_2 * SEq_4 - twoSEq_1 * SEq_3 - a_x;
    f_2 = twoSEq_1 * SEq_2 + twoSEq_3 * SEq_4 - a_y;
    f_3 = 1.0F - twoSEq_2 * SEq_2 - twoSEq_3 * SEq_3 - a_z;
    f_4 = twob_x * (0.5F - SEq_3 * SEq_3 - SEq_4 * SEq_4) + twob_z * (SEq_2SEq_4 - SEq_1SEq_3) - m_x;
    f_5 = twob_x * (SEq_2 * SEq_3 - SEq_1 * SEq_4) + twob_z * (SEq_1 * SEq_2 + SEq_3 * SEq_4) - m_y;
    f_6 = twob_x * (SEq_1SEq_3 + SEq_2SEq_4) + twob_z * (0.5F - SEq_2 * SEq_2 - SEq_3 * SEq_3) - m_z;
    J_11or24 = twoSEq_3; // J_11 negated in matrix multiplication
    J_12or23 = 2.0F * SEq_4;
    J_13or22 = twoSEq_1; // J_12 negated in matrix multiplication
    J_14or21 = twoSEq_2;
    J_32     = 2.0F * J_14or21; // negated in matrix multiplication
    J_33     = 2.0F * J_11or24; // negated in matrix multiplication
    J_41     = twob_zSEq_3;     // negated in matrix multiplication
    J_42     = twob_zSEq_4;
    J_43     = 2.0F * twob_xSEq_3 + twob_zSEq_1; // negated in matrix multiplication
    J_44     = 2.0F * twob_xSEq_4 - twob_zSEq_2; // negated in matrix multiplication
    J_51     = twob_xSEq_4 - twob_zSEq_2;        // negated in matrix multiplication
    J_52     = twob_xSEq_3 + twob_zSEq_1;
    J_53     = twob_xSEq_2 + twob_zSEq_4;
    J_54     = twob_xSEq_1 - twob_zSEq_3; // negated in matrix multiplication
    J_61     = twob_xSEq_3;
    J_62     = twob_xSEq_4 - 2.0F * twob_zSEq_2;
    J_63     = twob_xSEq_1 - 2.0F * twob_zSEq_3;
    J_64     = twob_xSEq_2;
    // compute the gradient (matrix multiplication)
    SEqHatDot_1 = J_14or21 * f_2 - J_11or24 * f_1 - J_41 * f_4 - J_51 * f_5 + J_61 * f_6;
    SEqHatDot_2 = J_12or23 * f_1 + J_13or22 * f_2 - J_32 * f_3 + J_42 * f_4 + J_52 * f_5 + J_62 * f_6;
    SEqHatDot_3 = J_12or23 * f_2 - J_33 * f_3 - J_13or22 * f_1 - J_43 * f_4 + J_53 * f_5 + J_63 * f_6;
    SEqHatDot_4 = J_14or21 * f_1 + J_11or24 * f_2 - J_44 * f_4 - J_54 * f_5 + J_64 * f_6;
    // normalise the gradient to estimate direction of the gyroscope error
    norm = sqrtf (SEqHatDot_1 * SEqHatDot_1 + SEqHatDot_2 * SEqHatDot_2 + SEqHatDot_3 * SEqHatDot_3 + SEqHatDot_4 * SEqHatDot_4);
    if (norm == 0.0F) {
        return false;
    }
    norm        = 1.0F / norm;
    SEqHatDot_1 = SEqHatDot_1 * norm;
    SEqHatDot_2 = SEqHatDot_2 * norm;
    SEqHatDot_3 = SEqHatDot_3 * norm;
    SEqHatDot_4 = SEqHatDot_4 * norm;
    // compute angular estimated direction of the gyroscope error
    w_err_x = twoSEq_1 * SEqHatDot_2 - twoSEq_2 * SEqHatDot_1 - twoSEq_3 * SEqHatDot_4 + twoSEq_4 * SEqHatDot_3;
    w_err_y = twoSEq_1 * SEqHatDot_3 + twoSEq_2 * SEqHatDot_4 - twoSEq_3 * SEqHatDot_1 - twoSEq_4 * SEqHatDot_2;
    w_err_z = twoSEq_1 * SEqHatDot_4 - twoSEq_2 * SEqHatDot_3 + twoSEq_3 * SEqHatDot_2 - twoSEq_4 * SEqHatDot_1;
    // compute and remove the gyroscope baises
    w_bx += w_err_x * dt * zeta;
    w_by += w_err_y * dt * zeta;
    w_bz += w_err_z * dt * zeta;
    w_x -= w_bx;
    w_y -= w_by;
    w_z -= w_bz;
    // compute the quaternion rate measured by gyroscopes
    SEqDot_omega_1 = -halfSEq_2 * w_x - halfSEq_3 * w_y - halfSEq_4 * w_z;
    SEqDot_omega_2 = halfSEq_1 * w_x + halfSEq_3 * w_z - halfSEq_4 * w_y;
    SEqDot_omega_3 = halfSEq_1 * w_y - halfSEq_2 * w_z + halfSEq_4 * w_x;
    SEqDot_omega_4 = halfSEq_1 * w_z + halfSEq_2 * w_y - halfSEq_3 * w_x;
    // compute then integrate the estimated quaternion rate
    SEq_1 += (SEqDot_omega_1 - (beta * SEqHatDot_1)) * dt;
    SEq_2 += (SEqDot_omega_2 - (beta * SEqHatDot_2)) * dt;
    SEq_3 += (SEqDot_omega_3 - (beta * SEqHatDot_3)) * dt;
    SEq_4 += (SEqDot_omega_4 - (beta * SEqHatDot_4)) * dt;
    // normalise quaternion
    norm = sqrtf (SEq_1 * SEq_1 + SEq_2 * SEq_2 + SEq_3 * SEq_3 + SEq_4 * SEq_4);
    if (norm == 0.0F) {
        return false;
    }
    norm = 1.0F / norm;
    SEq_1 *= norm;
    SEq_2 *= norm;
    SEq_3 *= norm;
    SEq_4 *= norm;
    // compute flux in the earth frame
    SEq_1SEq_2 = SEq_1 * SEq_2; // recompute axulirary variables
    SEq_1SEq_3 = SEq_1 * SEq_3;
    SEq_1SEq_4 = SEq_1 * SEq_4;
    SEq_3SEq_4 = SEq_3 * SEq_4;
    SEq_2SEq_3 = SEq_2 * SEq_3;
    SEq_2SEq_4 = SEq_2 * SEq_4;
    h_x = twom_x * (0.5F - SEq_3 * SEq_3 - SEq_4 * SEq_4) + twom_y * (SEq_2SEq_3 - SEq_1SEq_4) +
          twom_z * (SEq_2SEq_4 + SEq_1SEq_3);
    h_y = twom_x * (SEq_2SEq_3 + SEq_1SEq_4) + twom_y * (0.5F - SEq_2 * SEq_2 - SEq_4 * SEq_4) +
          twom_z * (SEq_3SEq_4 - SEq_1SEq_2);
    h_z = twom_x * (SEq_2SEq_4 - SEq_1SEq_3) + twom_y * (SEq_3SEq_4 + SEq_1SEq_2) +
          twom_z * (0.5F - SEq_2 * SEq_2 - SEq_3 * SEq_3);
    // normalise the flux vector to have only components in the x and z
    b_x = sqrtf ((h_x * h_x) + (h_y * h_y));
    b_z = h_z;

    pFilter->qEst.q1 = SEq_1;
    pFilter->qEst.q2 = SEq_2;
    pFilter->qEst.q3 = SEq_3;
    pFilter->qEst.q4 = SEq_4;
    pFilter->bx      = b_x;
    pFilter->bz      = b_z;
    pFilter->gbias.x = w_bx;
    pFilter->gbias.y = w_by;
    pFilter->gbias.z = w_bz;

    return true;
}

STATIC bool
FilterMadgwickWarmUp (FilterMadgwick_t* pFilter, vIMU_t* pIMU, vMag_t* pMag, uint32_t iterations, Vec3f* pOutAttitude) {

    if (pIMU == NULL) {
        pIMU = IMU_GetMutableActiveDevice ();
    }

    if (pMag == NULL) {
        pMag = Mag_GetMutableActiveDevice ();
    }

    float msStartTime = (float)GetMilliseconds ();
    RETURN_IF_NULL (pIMU, false, "No IMU device available");

    for (uint32_t i = 0; i < iterations; ++i) {

        Vec3f accel      = { 0.0F };
        Vec3f gyro       = { 0.0F };
        Vec3f mag        = { 0.0F };
        eSTATUS_t status = IMU_Update (pIMU, true, &accel, &gyro);
        RETURN_IF (status != eSTATUS_SUCCESS, false, "Failed to poll IMU");

        if (pMag != NULL) {
            status = Mag_Update (pMag, true, &mag);
            RETURN_IF (status != eSTATUS_SUCCESS, false, "Failed to poll Mag");
        }

        float dt = ((float)GetMilliseconds () - msStartTime) / 1000.0F; // Convert ms to seconds
        if (dt <= 0.0F) {
            continue;
        }
        msStartTime = (float)GetMilliseconds ();

        bool success = true;
        if (pMag != NULL) {
            success = FilterMadgwickUpdate (pFilter, &accel, &gyro, &mag, dt, pOutAttitude);
        } else {
            success = FilterMadgwickUpdate (pFilter, &accel, &gyro, NULL, dt, pOutAttitude);
        }
        RETURN_IF (success == false, false, "Madgwick update failed");
    }
    return true;
}

STATIC bool FilterMadgwickInit (FilterMadgwickInitConf_t conf, FilterMadgwick_t* pOut) {

    float gyroMeasureErrorDegs = conf.gyroMeasureErrorDegs;
    float gyroMeasureDriftDegs = conf.gyroMeasureDriftDegs;

    memset ((void*)pOut, 0, sizeof (FilterMadgwick_t));
    pOut->beta    = sqrtf (3.0F / 4.0F) * DEG2RAD (gyroMeasureErrorDegs);
    pOut->zeta    = sqrtf (3.0F / 4.0F) * DEG2RAD (gyroMeasureDriftDegs);
    pOut->qEst.q1 = 1.0F;
    pOut->qEst.q2 = 0.0F;
    pOut->qEst.q3 = 0.0F;
    pOut->qEst.q4 = 0.0F;
    return true;
}

STATIC bool
FilterMadgwickUpdate (FilterMadgwick_t* pFilter, Vec3f const* pAccel, Vec3f const* pGyro, Vec3f const* pMag, float dt, Vec3f* pOutAttitude) {

    if (pMag == NULL) {
        if (FilterMadgwickUpdate_6DOF (pFilter, pAccel, pGyro, dt) == false) {
            return false;
        }
    } else {
        if (FilterMadgwickUpdate_9DOF (pFilter, pAccel, pGyro, pMag, dt) == false) {
            return false;
        }
    }

    float q1 = pFilter->qEst.q1;
    float q2 = pFilter->qEst.q2;
    float q3 = pFilter->qEst.q3;
    float q4 = pFilter->qEst.q4;

    // Compute quaternion angles.
    // Then convert angles in radians to degrees.
    // Source: page 6 of Madgwick report
    /*
     * NOTE: atan2f returns values between -pi and +pi. It wraps at 0 --> +pi to -pi --> 0 .
     * Pitch and roll should never wrap but yaw can wrap.
     *
     * NOTE: atan and asin are NON reentrant
     */
    pOutAttitude->yaw =
    RAD2DEG (atan2f (2.0F * q2 * q3 - 2.0F * q1 * q4, 2.0F * q1 * q1 + 2.0F * q2 * q2 - 1));

    pOutAttitude->pitch = RAD2DEG (-asinf (2.0F * q2 * q4 + 2.0F * q1 * q3));

    pOutAttitude->roll =
    RAD2DEG (atan2f (2.0F * q3 * q4 - 2.0F * q1 * q2, 2.0F * q1 * q1 + 2.0F * q4 * q4 - 1.0F));

    return true;
}

eSTATUS_t FilterInit (FilterInitConf_t conf, Filter_t* pOut) {

    vFilter_t* pFilter = &gFilter;
    if (pOut != NULL) {
        pFilter = pOut;
    }

    memset ((void*)pFilter, 0, sizeof (vFilter_t));
    bool status = FilterMadgwickInit (conf.madgwickConf, &pFilter->madgwick);
    RETURN_IF (status != true, eSTATUS_FAILURE, "Failed to init Madgwick filter");

    pFilter->msLastUpdateTime = GetMilliseconds ();
    pFilter->isInitialized    = true;
    return eSTATUS_SUCCESS;
}

/*
 * If pOutAttitude is not NULL then the filters will warm up and return
 * the attitude in pOutAttitude.
 */
eSTATUS_t FilterStart (vFilter_t* pFilter, uint32_t warmUpIterations, Vec3f* pOutAttitude) {

    if (FILTER_VALID (pFilter) == false) {
        return eSTATUS_FAILURE;
    }

    bool success = true;
    if (pOutAttitude != NULL) {
        success &= FilterMadgwickWarmUp (&pFilter->madgwick, NULL, NULL, warmUpIterations, pOutAttitude);
    }

    return success ? eSTATUS_SUCCESS : eSTATUS_FAILURE;
}

eSTATUS_t FilterStop (vFilter_t* pFilter) {

    if (FILTER_VALID (pFilter) == false) {
        return eSTATUS_FAILURE;
    }
    return eSTATUS_SUCCESS;
}

eSTATUS_t
Filter_Update (vFilter_t* pFilter, Vec3f const* pAccel, Vec3f const* pGyro, Vec3f const* pMag, float dt, Vec3f* pOutput) {

    if (FILTER_VALID (pFilter) == false || pOutput == NULL) {
        return eSTATUS_FAILURE;
    }

    bool success = FilterMadgwickUpdate (&pFilter->madgwick, pAccel, pGyro, pMag, dt, pOutput);
    if (success == true) {
        pFilter->msLastUpdateTime = GetMilliseconds ();
    }
    return success ? eSTATUS_SUCCESS : eSTATUS_FAILURE;
}

vFilter_t const* FilterGetActiveFilter (void) {

    if (FILTER_VALID (&gFilter) == false) {
        return NULL;
    }
    return &gFilter;
}

vFilter_t* Filter_GetMutableActiveFilter (void) {

    if (FILTER_VALID (&gFilter) == false) {
        return NULL;
    }
    return &gFilter;
}