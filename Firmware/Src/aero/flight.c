#include <stdbool.h>
#include <stdint.h>

#include "common.h"

#include "aero/flight.h"

#include "core/core.h"

#include "drivers/sensors/sensor.h"

CFG_DEFINE (FlightCfg_t, FlightCfg) = {
    .gyroMeasureErrorDegs = 5.0F,
    .gyroMeasureDriftDegs = 0.0F,
};

FJ_DEFINE_SHARED (FlightData_t, e_FlightData);

typedef eSTATUS_t (*MadgwickUpdate_fn) (Madgwick_t* pMadgwick, Vec3f const* pAccel, Vec3f const* pGyro, Vec3f const* pMag, uint32_t usCurrentTime);

FJ_TESTABLE eSTATUS_t
Madgwick_Update_6DOF (Madgwick_t* pMadgwick, Vec3f const* pAccel, Vec3f const* pGyroDegs, Vec3f const* pUnused, uint32_t usCurrentTime) {

    FJ_UNUSED (pUnused);

    float dt                = US_TO_SECONDS (usCurrentTime - pMadgwick->usLastUpdate);
    pMadgwick->usLastUpdate = usCurrentTime;

    // convert degrees per second to radians per second
    float a_x  = pAccel->x;
    float a_y  = pAccel->y;
    float a_z  = pAccel->z;
    float w_x  = DEG2RAD (pGyroDegs->x);
    float w_y  = DEG2RAD (pGyroDegs->y);
    float w_z  = DEG2RAD (pGyroDegs->z);
    float beta = pMadgwick->beta;

    float SEq_1 = pMadgwick->qEst.q1;
    float SEq_2 = pMadgwick->qEst.q2;
    float SEq_3 = pMadgwick->qEst.q3;
    float SEq_4 = pMadgwick->qEst.q4;

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

    norm = 1.0F / (sqrtf (a_x * a_x + a_y * a_y + a_z * a_z) + 0.001F);
    // if (norm == 0.0F) {
    //     return false;
    // }
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

    norm = 1.0F / (sqrtf (SEq_1 * SEq_1 + SEq_2 * SEq_2 + SEq_3 * SEq_3 + SEq_4 * SEq_4) + 0.001F);
    SEq_1 *= norm;
    SEq_2 *= norm;
    SEq_3 *= norm;
    SEq_4 *= norm;

    pMadgwick->qEst.q1 = SEq_1;
    pMadgwick->qEst.q2 = SEq_2;
    pMadgwick->qEst.q3 = SEq_3;
    pMadgwick->qEst.q4 = SEq_4;

    return eSTATUS_SUCCESS;
}

FJ_TESTABLE eSTATUS_t
Madgwick_Update_9DOF (Madgwick_t* pMadgwick, Vec3f const* pAccel, Vec3f const* pGyro, Vec3f const* pMag, uint32_t usCurrentTime) {

    float dt                = US_TO_SECONDS (usCurrentTime - pMadgwick->usLastUpdate);
    pMadgwick->usLastUpdate = usCurrentTime;

    float SEq_1 = pMadgwick->qEst.q1;
    float SEq_2 = pMadgwick->qEst.q2;
    float SEq_3 = pMadgwick->qEst.q3;
    float SEq_4 = pMadgwick->qEst.q4;
    float b_x   = pMadgwick->bx;
    float b_z   = pMadgwick->bz;
    float beta  = pMadgwick->beta;
    float zeta  = pMadgwick->zeta;
    float w_bx  = pMadgwick->gbias.x;
    float w_by  = pMadgwick->gbias.y;
    float w_bz  = pMadgwick->gbias.z;
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
    norm = 1.0F / (sqrtf (a_x * a_x + a_y * a_y + a_z * a_z) + 0.001F);
    // if (norm == 0.0F) {
    //     return false;
    // }
    // norm = 1.0F / norm;
    a_x *= norm;
    a_y *= norm;
    a_z *= norm;
    // normalise the magnetometer measurement
    norm = 1.0F / (sqrtf (m_x * m_x + m_y * m_y + m_z * m_z) + 0.001F);
    // if (norm == 0.0F) {
    //     return false;
    // }
    // norm = 1.0F / norm;
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
    norm =
    1.0F /
    (sqrtf (SEqHatDot_1 * SEqHatDot_1 + SEqHatDot_2 * SEqHatDot_2 + SEqHatDot_3 * SEqHatDot_3 + SEqHatDot_4 * SEqHatDot_4) + 0.001F);
    // if (norm == 0.0F) {
    //     return false;
    // }
    // norm        = 1.0F / norm;
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
    norm = 1.0F / (sqrtf (SEq_1 * SEq_1 + SEq_2 * SEq_2 + SEq_3 * SEq_3 + SEq_4 * SEq_4) + 0.001F);
    // if (norm == 0.0F) {
    //     return false;
    // }
    // norm = 1.0F / norm;
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

    pMadgwick->qEst.q1 = SEq_1;
    pMadgwick->qEst.q2 = SEq_2;
    pMadgwick->qEst.q3 = SEq_3;
    pMadgwick->qEst.q4 = SEq_4;
    pMadgwick->bx      = b_x;
    pMadgwick->bz      = b_z;
    pMadgwick->gbias.x = w_bx;
    pMadgwick->gbias.y = w_by;
    pMadgwick->gbias.z = w_bz;

    return eSTATUS_SUCCESS;
}

FJ_TESTABLE void Madgwick_QuatToEuler (Vec4f const* pQuat, float outEuler[AXIS_IDX_COUNT]) {

    float q1 = pQuat->q1;
    float q2 = pQuat->q2;
    float q3 = pQuat->q3;
    float q4 = pQuat->q4;

    outEuler[AXIS_IDX_YAW] =
    RAD2DEG (atan2f (2.0F * q2 * q3 - 2.0F * q1 * q4, 2.0F * q1 * q1 + 2.0F * q2 * q2 - 1));
    outEuler[AXIS_IDX_PITCH] = RAD2DEG (-asinf (2.0F * q2 * q4 + 2.0F * q1 * q3));
    outEuler[AXIS_IDX_ROLL] =
    RAD2DEG (atan2f (2.0F * q3 * q4 - 2.0F * q1 * q2, 2.0F * q1 * q1 + 2.0F * q4 * q4 - 1.0F));
}

FJ_TESTABLE eSTATUS_t Madgwick_WarmUp (Madgwick_t* pMadgwick, uint32_t iterations, Vec3f* pOutAttitude) {

    uint32_t usStartTime       = GetMicroseconds ();
    MadgwickUpdate_fn fnUpdate = Madgwick_Update_6DOF;
    if (Mag_IsAvailable ()) {
        fnUpdate = Madgwick_Update_9DOF;
    }
    for (uint32_t i = 0; i < iterations; ++i) {

        Acc_Update (true);
        Gyro_Update (true);
        Mag_Update (true);

        fnUpdate (
        pMadgwick,
        &Acc_GetMutable ()->filteredData,
        &Gyro_GetMutable ()->filteredData,
        &Mag_GetMutable ()->filteredData,
        usStartTime
        );

        Madgwick_QuatToEuler (&pMadgwick->qEst, pOutAttitude);
        usStartTime = GetMicroseconds ();
    }
    return eSTATUS_SUCCESS;
}

FJ_TESTABLE eSTATUS_t Madgwick_Init (FlightCfg_t const* pCfg, Madgwick_t* pOut) {

    float gyroMeasureErrorDegs = pCfg->gyroMeasureErrorDegs;
    float gyroMeasureDriftDegs = pCfg->gyroMeasureDriftDegs;
    pOut->beta                 = sqrtf (3.0F / 4.0F) * DEG2RAD (gyroMeasureErrorDegs);
    pOut->zeta                 = sqrtf (3.0F / 4.0F) * DEG2RAD (gyroMeasureDriftDegs);
    pOut->qEst.q1              = 1.0F;
    pOut->qEst.q2              = 0.0F;
    pOut->qEst.q3              = 0.0F;
    pOut->qEst.q4              = 0.0F;
    pOut->usLastUpdate         = GetMicroseconds ();
    return true;
}

eSTATUS_t FlightData_Init (bool doWarmUp) {

    RETURN_IF (FJ_FAIL (Madgwick_Init (FlightCfg_Get (), &e_FlightData.madgwick)), eSTATUS_FAILURE, "Madgwick filter initialization failed");
    if (doWarmUp) {
        RETURN_IF (FJ_FAIL (Madgwick_WarmUp (&e_FlightData.madgwick, 500, &e_FlightData.currentAttitude)), eSTATUS_FAILURE, "Madgwick filter warm-up failed");
    }

    return eSTATUS_SUCCESS;
}

eSTATUS_t Attitude_Update (uint32_t usCurrentTime) {

    // TODO should update target attitude here as well

    MadgwickUpdate_fn fnUpdate = Madgwick_Update_6DOF;
    if (Mag_IsAvailable ()) {
        fnUpdate = Madgwick_Update_9DOF;
    }

    eSTATUS_t status = fnUpdate (
    &e_FlightData.madgwick,
    &Acc_GetMutable ()->filteredData,
    &Gyro_GetMutable ()->filteredData,
    &Mag_GetMutable ()->filteredData,
    usCurrentTime
    );

    RETURN_IF (FJ_FAIL (status), status, "Attitude update failed");
    Madgwick_QuatToEuler (&e_FlightData.madgwick.qEst, e_FlightData.currentAttitude);
    return eSTATUS_SUCCESS;
}