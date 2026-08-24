#include "hal.h"
#include "target.h"

#include "common/mem.h"

#include "core/core.h"

#include "common/filter.h"

#include <math.h>
#include <stdbool.h>
#include <string.h>

/*
 * Accelerometer trust band, as a fraction of 1 g.
 *
 * The accel is only a gravity reference when the vehicle is NOT accelerating.
 * For a rotorcraft in free flight specific force points along the thrust axis
 * whatever the attitude, so the reading stays plausible while carrying no
 * attitude information at all - which is why this is a band and not a validity
 * test. What it does catch is the case where the vehicle is banked far enough
 * that holding altitude needs g/cos(phi) of thrust: at the 1.1 bound that is
 * 24.6 deg, and past there the reading is provably not gravity.
 *
 * The band is Betaflight's (imuIsAccelerometerHealthy, imu.c) and is a physical
 * property of "is this reading gravity", not an airframe tune - hence a define
 * here rather than a cfg.h knob.
 */
#define FILTER_ACCEL_TRUST_MIN_G 0.9F
#define FILTER_ACCEL_TRUST_MAX_G 1.1F
#define FILTER_GRAVITY_MPS2      9.80665F

/* False when the accel magnitude says the reading cannot be gravity. Also false
 * for NaN, which is the point: both comparisons fail and the sample is dropped
 * rather than integrated. */
STATIC INLINE bool MadgwickFilter_AccelUsable (float magMps2) {
    float g = magMps2 / FILTER_GRAVITY_MPS2;
    return (g > FILTER_ACCEL_TRUST_MIN_G) && (g < FILTER_ACCEL_TRUST_MAX_G);
}

/*
 * The filter expects the accel and gyro data to be in the FRD coordinate
 * system. For example, an accel reading of (0, 0, +1g) means the sensor is
 * stationary and level. So the returned attitude will move towards (0, 0,
 * 0). If the IMU is stationary and level but has an accel reading of (0,
 * 1g, 0) then the returned attitude will move towards (0, -90, 0).
 * Which is incorrect so the IMU data needs to be in FRD frame.
 */
static inline void MadgwickFilter_Update_6DOF_ (MadgwickFilter_t* pFilter, Vec3f const* pAccel, Vec3f const* pGyroDegs, float dt) {
    // Source: https://courses.cs.washington.edu/courses/cse474/17wi/labs/l4/madgwick_internal_report.pdf

    // convert degrees per second to radians per second
    float a_x  = pAccel->x;
    float a_y  = pAccel->y;
    float a_z  = pAccel->z;
    float w_x  = DEG2RAD (pGyroDegs->x);
    float w_y  = DEG2RAD (pGyroDegs->y);
    float w_z  = DEG2RAD (pGyroDegs->z);
    float beta = pFilter->beta * pFilter->accelTrust;

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

    float accelMag = sqrtf (a_x * a_x + a_y * a_y + a_z * a_z);
    norm = accelMag + 0.0001F;
    norm = 1.0F / norm;
    a_x *= norm;
    a_y *= norm;
    a_z *= norm;

    /* With no magnetometer the accel is the only correction there is, so an
     * untrustworthy one leaves nothing to apply - coast on the gyro. The 9DOF
     * path drops just the accel rows instead, because its mag rows survive. */
    if (!MadgwickFilter_AccelUsable (accelMag)) {
        beta = 0.0F;
    }

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
}

static inline void MadgwickFilter_Update_9DOF_ (MadgwickFilter_t* pFilter, Vec3f const* pAccel, Vec3f const* pGyro, Vec3f const* pMag, float dt) {

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
    float accelMag = sqrtf (a_x * a_x + a_y * a_y + a_z * a_z);
    norm = accelMag + 0.0001F;
    norm = 1.0F / norm;
    a_x *= norm;
    a_y *= norm;
    a_z *= norm;

    if (!MadgwickFilter_AccelUsable (accelMag)) {
        beta = 0.0F;
    }

    // normalise the magnetometer measurement
    norm = sqrtf (m_x * m_x + m_y * m_y + m_z * m_z) + 0.0001F;
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
    /* f_1..f_3 are the accel rows of the objective function and f_4..f_6 the mag
     * rows; the gradient below is linear in both, so scaling or zeroing the
     * first three moves the accel's contribution and leaves the heading
     * correction intact. That is the same split Betaflight makes - it gates the
     * accel term out of the Mahony error while the mag/COG term still reaches
     * the same gain.
     *
     * accelTrust therefore rides HERE and not on beta, which is a single gain
     * over the combined gradient and would drag heading down with it. The 6DOF
     * path has to put the same scaling on beta instead: with only these three
     * rows in the gradient, normalising it divides any common factor straight
     * back out, so scaling them there would do exactly nothing. */
    f_1 *= pFilter->accelTrust;
    f_2 *= pFilter->accelTrust;
    f_3 *= pFilter->accelTrust;
    
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
    // clang-format off
    norm = sqrtf (SEqHatDot_1 * SEqHatDot_1 + SEqHatDot_2 * SEqHatDot_2 + SEqHatDot_3 * SEqHatDot_3 + SEqHatDot_4 * SEqHatDot_4) + 0.0001F;
    // clang-format on
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
    norm = sqrtf (SEq_1 * SEq_1 + SEq_2 * SEq_2 + SEq_3 * SEq_3 + SEq_4 * SEq_4) + 0.0001F;
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
}

eSTATUS_t
MadgwickFilter_Update (MadgwickFilter_t* pFilter, Vec3f const* pAccel, Vec3f const* pGyro, Vec3f const* pMag, float dt, Vec3f* pOutAttitude) {

    if (!pFilter || !pAccel || !pGyro || dt <= 0.0F) {
        return eSTATUS_FAILURE;
    }

    if (!pMag) {
        MadgwickFilter_Update_6DOF_ (pFilter, pAccel, pGyro, dt);
    } else {
        MadgwickFilter_Update_9DOF_ (pFilter, pAccel, pGyro, pMag, dt);
    }

    if (pOutAttitude) {
        MadgwickFilter_QuatToEuler (pFilter, pOutAttitude);
    }
    pFilter->usLastUpdateTime = GetMicroseconds ();
    return eSTATUS_SUCCESS;
}

void MadgwickFilter_QuatToEuler (MadgwickFilter_t const* pFilter, Vec3f* pOutEuler) {

    if (!pFilter || !pOutEuler) {
        return;
    }

    float q1 = pFilter->qEst.q1;
    float q2 = pFilter->qEst.q2;
    float q3 = pFilter->qEst.q3;
    float q4 = pFilter->qEst.q4;

    // Compute quaternion angles.
    // Then convert angles in radians to degrees.
    /*
     * Page 6 of the Madgwick report gives these for (S)(E)q - the Earth frame
     * expressed relative to the sensor frame. Flight code wants the opposite:
     * the body attitude relative to Earth, (E)(S)q, which is the conjugate.
     * Using the report's formulas verbatim negates all three angles.
     *
     * Substituting the conjugate (q1, -q2, -q3, -q4) flips the sign of the
     * second term in each expression, which is what appears below. The roll
     * form reduces to the standard aerospace ZYX extraction,
     * atan2(2(q1*q2 + q3*q4), 1 - 2(q2^2 + q3^2)).
     *
     * NOTE: atan2f returns values between -pi and +pi. It wraps at 0 --> +pi to -pi --> 0 .
     * Pitch and roll should never wrap but yaw can wrap.
     *
     * NOTE: atan and asin are NON reentrant
     */
    pOutEuler->yaw = RAD2DEG (atan2f (2.0F * q2 * q3 + 2.0F * q1 * q4, 2.0F * q1 * q1 + 2.0F * q2 * q2 - 1));

    pOutEuler->pitch = RAD2DEG (-asinf (2.0F * q2 * q4 - 2.0F * q1 * q3));

    pOutEuler->roll =
    RAD2DEG (atan2f (2.0F * q3 * q4 + 2.0F * q1 * q2, 2.0F * q1 * q1 + 2.0F * q4 * q4 - 1.0F));
}

eSTATUS_t MadgwickFilter_Init (MadgwickFilter_t* pFilter) {

    if (!pFilter) {
        return eSTATUS_FAILURE;
    }
    pFilter->usLastUpdateTime = GetMicroseconds ();
    pFilter->bx               = 1.0F;
    pFilter->bz               = 0.0F;
    pFilter->accelTrust       = 1.0F;
    pFilter->beta             = sqrtf (3.0F / 4.0F) * DEG2RAD (pFilter->cfg.gyroMeasureErrorDegs);
    pFilter->zeta             = sqrtf (3.0F / 4.0F) * DEG2RAD (pFilter->cfg.gyroMeasureDriftDegs);
    pFilter->qEst.q1          = 1.0F;
    pFilter->qEst.q2          = 0.0F;
    pFilter->qEst.q3          = 0.0F;
    pFilter->qEst.q4          = 0.0F;
    return eSTATUS_SUCCESS;
}

/*
 * Clears the STATE and leaves cfg alone, so it doubles as the reset a caller
 * needs after a discontinuity (arming, a mode change) without having to
 * re-supply the cutoff. Pid_ResetAxis relies on that.
 */
eSTATUS_t LowPassFilter_Init (LowPassFilter_t* pFilter) {

    if (!pFilter) {
        return eSTATUS_FAILURE;
    }
    pFilter->state    = 0.0F;
    pFilter->hasState = false;
    return eSTATUS_SUCCESS;
}

float LowPassFilter_Update (LowPassFilter_t* pFilter, float input, float dt) {

    /* Disabled, or a dt that cannot produce a meaningful alpha: pass the input
     * through rather than invent state from it. `!(x > 0)` rather than `x <= 0`
     * so NaN takes these branches too - same idiom, and same reason, as the dt
     * guard in Pid_UpdateAxis. */
    if (!pFilter || !(pFilter->cfg.cutoffHz > 0.0F) || !(dt > 0.0F)) {
        return input;
    }

    /* Seed from the first sample rather than from zero. Starting at zero makes
     * the output ramp from the origin over a full time constant, and on the D
     * path that ramp reads as a large false derivative for exactly as long -
     * the same spike PidAxis_t.hasPrevMeasurement exists to suppress, just
     * spread over more frames. */
    if (!pFilter->hasState) {
        pFilter->state    = input;
        pFilter->hasState = true;
        return input;
    }

    float rc    = 1.0F / (6.283185307179586F * pFilter->cfg.cutoffHz); // 1 / (2π·fc)
    float alpha = dt / (dt + rc);
    pFilter->state += alpha * (input - pFilter->state);
    return pFilter->state;
}

eSTATUS_t HorizontalFilter_Init (HorizontalFilter_t* pFilter) {

    if (!pFilter || pFilter->cfg.kPos < 0.0F || pFilter->cfg.kVel < 0.0F ||
        pFilter->cfg.kBias < 0.0F || pFilter->cfg.maxBias <= 0.0F) {
        return eSTATUS_FAILURE;
    }
    for (int i = 0; i < HORIZ_AXES; i++) {
        pFilter->pos[i]       = 0.0F;
        pFilter->vel[i]       = 0.0F;
        pFilter->accelBias[i] = 0.0F;
    }
    return eSTATUS_SUCCESS;
}

eSTATUS_t HorizontalFilter_Predict (HorizontalFilter_t* pFilter, float const* accelNed, float dt) {

    if (!pFilter || !accelNed || dt <= 0.0F) {
        return eSTATUS_FAILURE;
    }
    for (int i = 0; i < HORIZ_AXES; i++) {
        float accel = accelNed[i] - pFilter->accelBias[i];
        pFilter->pos[i] += (pFilter->vel[i] * dt) + (0.5F * accel * dt * dt);
        pFilter->vel[i] += accel * dt;
    }
    return eSTATUS_SUCCESS;
}

eSTATUS_t HorizontalFilter_Correct (HorizontalFilter_t* pFilter, float const* posNed,
                                    float const* velNed, float dt) {

    if (!pFilter || !posNed || !velNed || dt <= 0.0F) {
        return eSTATUS_FAILURE;
    }

    for (int i = 0; i < HORIZ_AXES; i++) {
        pFilter->pos[i] += pFilter->cfg.kPos * (posNed[i] - pFilter->pos[i]) * dt;

        /* Velocity carries the bias, not position. Same argument as the header:
         * the receiver's velocity is the better-conditioned measurement by more
         * than an order of magnitude, and the bias is the state worth spending
         * it on. */
        float velError = velNed[i] - pFilter->vel[i];
        pFilter->vel[i] += pFilter->cfg.kVel * velError * dt;

        /* Estimated velocity below the measured one (error > 0) means the accel
         * path is under-integrating, i.e. the bias being subtracted is too
         * large - so the bias moves opposite the error, exactly as the vertical
         * filter's does against its altitude error. */
        pFilter->accelBias[i] -= pFilter->cfg.kBias * velError * dt;

        if (pFilter->accelBias[i] > pFilter->cfg.maxBias) {
            pFilter->accelBias[i] = pFilter->cfg.maxBias;
        } else if (pFilter->accelBias[i] < -pFilter->cfg.maxBias) {
            pFilter->accelBias[i] = -pFilter->cfg.maxBias;
        }
    }
    return eSTATUS_SUCCESS;
}

eSTATUS_t AltitudeFilter_Init (AltitudeFilter_t* pFilter) {

    if (!pFilter || pFilter->cfg.kAlt < 0.0F || pFilter->cfg.kVel < 0.0F ||
        pFilter->cfg.kBias < 0.0F || pFilter->cfg.maxBias <= 0.0F) {
        return eSTATUS_FAILURE;
    }
    pFilter->alt       = 0.0F;
    pFilter->vz        = 0.0F;
    pFilter->accelBias = 0.0F;
    return eSTATUS_SUCCESS;
}

/*
 * Dead-reckon the vertical state forward one IMU sample. accelUp is UP-positive
 * acceleration with gravity already removed - see Nav_VerticalAccelUp(), which
 * owns the frame rotation and the sign convention.
 *
 * The 0.5*a*dt^2 term is not decoration: at 400 Hz it is small, but this is the
 * only place altitude gains any information faster than the baro's 50 Hz, and
 * dropping it biases every climb and descent in the same direction.
 */
eSTATUS_t AltitudeFilter_Predict (AltitudeFilter_t* pFilter, float accelUp, float dt) {

    if (!pFilter || dt <= 0.0F) {
        return eSTATUS_FAILURE;
    }
    float accel = accelUp - pFilter->accelBias;
    pFilter->alt += (pFilter->vz * dt) + (0.5F * accel * dt * dt);
    pFilter->vz += accel * dt;
    return eSTATUS_SUCCESS;
}

/*
 * Pull the state towards a fresh baro altitude. dt is the interval since the
 * LAST baro sample, not since the last predict - the gains are per second of
 * correction, so feeding the IMU's dt here would scale every gain by the rate
 * ratio and turn a 1 s time constant into an oscillation.
 *
 * Call this only on iterations where a new sample actually arrived. Re-applying
 * the same measurement does not average out; it just multiplies the gain.
 */
eSTATUS_t AltitudeFilter_Correct (AltitudeFilter_t* pFilter, float baroAlt, float dt) {

    if (!pFilter || dt <= 0.0F) {
        return eSTATUS_FAILURE;
    }

    float error = baroAlt - pFilter->alt;
    pFilter->alt += pFilter->cfg.kAlt * error * dt;
    pFilter->vz += pFilter->cfg.kVel * error * dt;

    /* Estimated altitude below the baro's (error > 0) means the accel path is
     * under-integrating, i.e. the bias being subtracted is too large - so the
     * bias moves opposite the error. */
    pFilter->accelBias -= pFilter->cfg.kBias * error * dt;

    if (pFilter->accelBias > pFilter->cfg.maxBias) {
        pFilter->accelBias = pFilter->cfg.maxBias;
    } else if (pFilter->accelBias < -pFilter->cfg.maxBias) {
        pFilter->accelBias = -pFilter->cfg.maxBias;
    }
    return eSTATUS_SUCCESS;
}
