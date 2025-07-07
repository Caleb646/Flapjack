#include "motion_control/filter.h"
#include "hal.h"
#include "log.h"
#include "sensors/imu/imu.h"
#include <math.h>
#include <string.h>


/*
 *  The initial sensor frame (x, y, z) needs to be aligned with the
 *  world frame (X, Y, Z). For example if the world frame is (0, 0, 1 [gravity])
 *  then the initial sensor frame should be [using accelerometer data] (0, 0, 9.8m/s^2).
 *  If not aligned, the filter will show the imu as having rotated when it has not moved.
 */

STATUS_TYPE
FilterMadgwick6DOF (
FilterMadgwickContext* pContext,
Vec3f const* pAccel,
Vec3f const* pGyroDegs,
float dt,
Vec3f* pOutputAttitude) {
    // Source: https://courses.cs.washington.edu/courses/cse474/17wi/labs/l4/madgwick_internal_report.pdf
    if (pContext == NULL || pAccel == NULL || pGyroDegs == NULL || pOutputAttitude == NULL) {
        return eSTATUS_FAILURE;
    }

    // convert degrees per second to radians per second
    float w_x = DEG2RAD (pGyroDegs->x);
    float w_y = DEG2RAD (pGyroDegs->y);
    float w_z = DEG2RAD (pGyroDegs->z);

    float a_x = pAccel->x;
    float a_y = pAccel->y;
    float a_z = pAccel->z;

    Vec4f* pCurEstimate = &pContext->est;
    float beta          = pContext->beta;

    float SEq_1 = pCurEstimate->q1;
    float SEq_2 = pCurEstimate->q2;
    float SEq_3 = pCurEstimate->q3;
    float SEq_4 = pCurEstimate->q4;

    float norm           = 0.0F;
    float SEqDot_omega_1 = 0.0F;
    float SEqDot_omega_2 = 0.0F;
    float SEqDot_omega_3 = 0.0F;
    float SEqDot_omega_4 = 0.0F; // quaternion derrivative from gyroscopes elements float
    float f_1         = 0.0F;
    float f_2         = 0.0F;
    float f_3         = 0.0F; // objective function elements
    float J_11or24    = 0.0F;
    float J_12or23    = 0.0F;
    float J_13or22    = 0.0F;
    float J_14or21    = 0.0F;
    float J_32        = 0.0F;
    float J_33        = 0.0F;
    float SEqHatDot_1 = 0.0F;
    float SEqHatDot_2 = 0.0F;
    float SEqHatDot_3 = 0.0F;
    float SEqHatDot_4 = 0.0F; // Axulirary variables to avoid reapeated calcualtions
    float halfSEq_1 = 0.5F * SEq_1;
    float halfSEq_2 = 0.5F * SEq_2;
    float halfSEq_3 = 0.5F * SEq_3;
    float halfSEq_4 = 0.5F * SEq_4;
    float twoSEq_1  = 2.0F * SEq_1;
    float twoSEq_2  = 2.0F * SEq_2;
    float twoSEq_3  = 2.0F * SEq_3;

    if (!((a_x == 0.0F) && (a_y == 0.0F) && (a_z == 0.0F))) {
        norm = 1.0F / sqrtf (a_x * a_x + a_y * a_y + a_z * a_z);
        a_x *= norm;
        a_y *= norm;
        a_z *= norm;
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

    float factor = sqrtf (
    SEqHatDot_1 * SEqHatDot_1 + SEqHatDot_2 * SEqHatDot_2 +
    SEqHatDot_3 * SEqHatDot_3 + SEqHatDot_4 * SEqHatDot_4);
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

    pCurEstimate->q1 = SEq_1;
    pCurEstimate->q2 = SEq_2;
    pCurEstimate->q3 = SEq_3;
    pCurEstimate->q4 = SEq_4;

    float q1 = SEq_1;
    float q2 = SEq_2;
    float q3 = SEq_3;
    float q4 = SEq_4;

    // Compute quaternion angles.
    // Then convert angles in radians to degrees.
    // NOTE: page 6 of Madgwick report
    // Roll is about the x axis, represented as phi
    // Pitch is about the y axis, represented as theta
    // Yaw is about the z axis, represented as psi
    pOutputAttitude->yaw = RAD2DEG (
    atan2f (2.0F * q2 * q3 - 2.0F * q1 * q4, 2.0F * q1 * q1 + 2.0F * q2 * q2 - 1));

    pOutputAttitude->pitch = RAD2DEG (-asinf (2.0F * q2 * q4 + 2.0F * q1 * q3));

    pOutputAttitude->roll = RAD2DEG (
    atan2f (2.0F * q3 * q4 - 2.0F * q1 * q2, 2.0F * q1 * q1 + 2.0F * q4 * q4 - 1.0F));

    return eSTATUS_SUCCESS;
}

// STATUS_TYPE
// FilterMadgwick9DOF (FilterMadgwickContext* pContext, Vec3f accel, Vec3f gyro, Vec3f magno, Vec3f* pOutputAttitude) {
//     return eSTATUS_SUCCESS;
// }

STATUS_TYPE
FilterMadgwickWarmUp (
uint32_t iterations,
IMU* pIMU,
float expectedGyroErrorDegs,
float warmUpBeta,
FilterMadgwickContext* pOutContext) {

    FilterMadgwickContext context = { 0 };
    context.beta                  = warmUpBeta;
    context.est.q1                = 1.0F;
    context.est.q2                = 0.0F;
    context.est.q3                = 0.0F;
    context.est.q4                = 0.0F;
    Vec3f attitude                = { 0.0F };
    float startTime               = (float)HAL_GetTick ();

    if (pIMU == NULL || pOutContext == NULL) {
        LOG_ERROR ("Invalid parameters: pIMU or pOutContext is NULL");
        return eSTATUS_FAILURE;
    }

    for (uint32_t i = 0U; i < iterations; ++i) {
        Vec3f accel        = { 0.0F };
        Vec3f gyro         = { 0.0F };
        STATUS_TYPE status = IMUPollData (pIMU, &accel, &gyro);

        if (status != eSTATUS_SUCCESS) {
            LOG_ERROR ("IMUPollData failed with status: [%d] at iteration [%u]", status, (uint16_t)i);
            return status;
        }

        float dt = (HAL_GetTick () - startTime) / 1000.0F; // Convert ms to seconds

        if (dt <= 0.0F) {
            continue;
        }

        startTime = HAL_GetTick ();

        status = FilterMadgwick6DOF (&context, &accel, &gyro, dt, &attitude);
        if (status != eSTATUS_SUCCESS) {
            LOG_ERROR ("FilterMadgwick6DOF failed with status: %d", status);
            return status;
        }
    }

    // Copy the final filter estimate to the output context
    pOutContext->beta = sqrtf (3.0F / 4.0F) * DEG2RAD (expectedGyroErrorDegs);
    pOutContext->est.q1 = context.est.q1;
    pOutContext->est.q2 = context.est.q2;
    pOutContext->est.q3 = context.est.q3;
    pOutContext->est.q4 = context.est.q4;
    LOG_INFO ("Madgwick finished warmup");
    LOG_DATA_CURRENT_ATTITUDE (attitude);
    return eSTATUS_SUCCESS;
}

// STATUS_TYPE FilterMadgwickInit (FilterMadgwickContext* pContext, float gyroMeasureErrorDegs) {
//     // Initialization values: https://courses.cs.washington.edu/courses/cse474/17wi/labs/l4/madgwick_internal_report.pdf
//     memset ((void*)pContext, 0, sizeof (FilterMadgwickContext));
//     pContext->beta = sqrtf (3.0F / 4.0F) * DEG2RAD (gyroMeasureErrorDegs);
//     pContext->est.q1 = 1.0F;
//     pContext->est.q2 = 0.0F;
//     pContext->est.q3 = 0.0F;
//     pContext->est.q4 = 0.0F;
//     return eSTATUS_SUCCESS;
// }