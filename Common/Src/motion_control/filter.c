#include "motion_control/filter.h"
#include <math.h>
#include <string.h>

STATUS_TYPE
FilterMadgwick6DOF (FilterMadgwickContext* pContext, Vec3f gyro, Vec3f accel, Vec3f* pOutputAttitude) {
    // Source: https://courses.cs.washington.edu/courses/cse474/17wi/labs/l4/madgwick_internal_report.pdf

    // convert degrees per second to radians per second
    float w_x = gyro.x * 0.017453;
    float w_y = gyro.y * 0.017453;
    float w_z = gyro.z * 0.017453;

    float a_x = accel.x;
    float a_y = accel.y;
    float a_z = accel.z;

    float dt   = pContext->dt;
    float beta = pContext->beta;

    float SEq_1 = pContext->seq1;
    float SEq_2 = pContext->seq2;
    float SEq_3 = pContext->seq3;
    float SEq_4 = pContext->seq4;

    float norm;
    float SEqDot_omega_1, SEqDot_omega_2, SEqDot_omega_3,
    SEqDot_omega_4; // quaternion derrivative from gyroscopes elements float
    float f_1, f_2, f_3; // objective function elements
    float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33;
    float SEqHatDot_1, SEqHatDot_2, SEqHatDot_3,
    SEqHatDot_4; // Axulirary variables to avoid reapeated calcualtions
    float halfSEq_1 = 0.5f * SEq_1;
    float halfSEq_2 = 0.5f * SEq_2;
    float halfSEq_3 = 0.5f * SEq_3;
    float halfSEq_4 = 0.5f * SEq_4;
    float twoSEq_1  = 2.0f * SEq_1;
    float twoSEq_2  = 2.0f * SEq_2;
    float twoSEq_3  = 2.0f * SEq_3;

    if (!((a_x == 0.0f) && (a_y == 0.0f) && (a_z == 0.0f))) {
        norm = 1.0f / sqrt (a_x * a_x + a_y * a_y + a_z * a_z);
        a_x *= norm;
        a_y *= norm;
        a_z *= norm;
    }

    // Compute the objective function and Jacobian
    f_1 = twoSEq_2 * SEq_4 - twoSEq_1 * SEq_3 - a_x;
    f_2 = twoSEq_1 * SEq_2 + twoSEq_3 * SEq_4 - a_y;
    f_3 = 1.0f - twoSEq_2 * SEq_2 - twoSEq_3 * SEq_3 - a_z;

    J_11or24 = twoSEq_3;
    J_12or23 = 2.0f * SEq_4;
    J_13or22 = twoSEq_1;
    J_14or21 = twoSEq_2;
    J_32     = 2.0f * J_14or21;
    J_33     = 2.0f * J_11or24;

    SEqHatDot_1 = J_14or21 * f_2 - J_11or24 * f_1;
    SEqHatDot_2 = J_12or23 * f_1 + J_13or22 * f_2 - J_32 * f_3;
    SEqHatDot_3 = J_12or23 * f_2 - J_33 * f_3 - J_13or22 * f_1;
    SEqHatDot_4 = J_14or21 * f_1 + J_11or24 * f_2;

    // negated in matrix multiplication
    norm = 1.0f / sqrt (
                  SEqHatDot_1 * SEqHatDot_1 + SEqHatDot_2 * SEqHatDot_2 +
                  SEqHatDot_3 * SEqHatDot_3 + SEqHatDot_4 * SEqHatDot_4);
    SEqHatDot_1 *= norm;
    SEqHatDot_2 *= norm;
    SEqHatDot_3 *= norm;
    SEqHatDot_4 *= norm;

    SEqDot_omega_1 = -halfSEq_2 * w_x - halfSEq_3 * w_y - halfSEq_4 * w_z;
    SEqDot_omega_2 = halfSEq_1 * w_x + halfSEq_3 * w_z - halfSEq_4 * w_y;
    SEqDot_omega_3 = halfSEq_1 * w_y - halfSEq_2 * w_z + halfSEq_4 * w_x;
    SEqDot_omega_4 = halfSEq_1 * w_z + halfSEq_2 * w_y - halfSEq_3 * w_x;

    SEq_1 += (SEqDot_omega_1 - (beta * SEqHatDot_1)) * dt;
    SEq_2 += (SEqDot_omega_2 - (beta * SEqHatDot_2)) * dt;
    SEq_3 += (SEqDot_omega_3 - (beta * SEqHatDot_3)) * dt;
    SEq_4 += (SEqDot_omega_4 - (beta * SEqHatDot_4)) * dt;

    norm = 1.0f / sqrt (SEq_1 * SEq_1 + SEq_2 * SEq_2 + SEq_3 * SEq_3 + SEq_4 * SEq_4);
    SEq_1 *= norm;
    SEq_2 *= norm;
    SEq_3 *= norm;
    SEq_4 *= norm;

    pContext->seq1 = SEq_1;
    pContext->seq2 = SEq_2;
    pContext->seq3 = SEq_3;
    pContext->seq4 = SEq_4;

    float q0 = SEq_1;
    float q1 = SEq_2;
    float q2 = SEq_3;
    float q3 = SEq_4;

    // Compute quaternion angles.
    // Then convert angles in radians to degrees.
    // North-West-Up (NWU)
    // x-axis pointing to the North
    // y-axis pointing to the West
    // z-axis pointing away from the surface of the Earth
    pOutputAttitude->roll =
    atan2 (q0 * q1 + q2 * q3, 0.5f - q1 * q1 - q2 * q2) * 57.29577951f;
    pOutputAttitude->pitch =
    -asin (clipf32 (-2.0f * (q1 * q3 - q0 * q2), -0.999999f, 0.999999f)) * 57.29577951f;
    pOutputAttitude->yaw =
    -atan2 (q1 * q2 + q0 * q3, 0.5f - q2 * q2 - q3 * q3) * 57.29577951f;

    return eSTATUS_SUCCESS;
}

STATUS_TYPE
FilterMadgwick9DOF (FilterMadgwickContext* pContext, Vec3f gyro, Vec3f accel, Vec3f magno, Vec3f* pOutputAttitude) {
    return eSTATUS_SUCCESS;
}

STATUS_TYPE FilterMadgwickInit (FilterMadgwickContext* pContext) {
    // Initialization values: https://courses.cs.washington.edu/courses/cse474/17wi/labs/l4/madgwick_internal_report.pdf
    memset ((void*)pContext, 0, sizeof (FilterMadgwickContext));
    pContext->dt               = 0.001f;
    pContext->gyroMeasureError = M_PI * (5.0f / 180.0f);
    pContext->beta = sqrt (3.0f / 4.0f) * pContext->gyroMeasureError;
    pContext->seq1 = 1.0f;
    pContext->seq2 = 0.0f;
    pContext->seq3 = 0.0f;
    pContext->seq4 = 0.0f;
    return eSTATUS_SUCCESS;
}