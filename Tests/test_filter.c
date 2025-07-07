#include "common.h"
#include "motion_control/filter.h"
#include "unity/unity.h"
#include <math.h>

#ifndef UNIT_TEST
#error "UNIT_TEST should be defined in this file"
#endif

#define PI_F 3.14159265359F


void test_FilterMadgwickInit (void) {
    FilterMadgwickContext context;
    float gyroMeasureErrorDegs = 5.0F;

    STATUS_TYPE status = FilterMadgwickInit (&context, gyroMeasureErrorDegs, NULL);
    TEST_ASSERT_EQUAL_INT (eSTATUS_SUCCESS, status);

    // Check that quaternion is initialized to identity (no rotation)
    TEST_ASSERT_FLOAT_WITHIN (0.001F, 1.0F, context.est.q1);
    TEST_ASSERT_FLOAT_WITHIN (0.001F, 0.0F, context.est.q2);
    TEST_ASSERT_FLOAT_WITHIN (0.001F, 0.0F, context.est.q3);
    TEST_ASSERT_FLOAT_WITHIN (0.001F, 0.0F, context.est.q4);

    // Check that beta is calculated correctly
    float expected_beta = sqrt (3.0F / 4.0F) * (gyroMeasureErrorDegs * PI_F / 180.0F);
    TEST_ASSERT_FLOAT_WITHIN (0.001F, expected_beta, context.beta);
}

void test_FilterMadgwick6DOF_NullPointers (void) {
    FilterMadgwickContext context;
    Vec3f accel = { 0.0F, 0.0F, 9.81F };
    Vec3f gyro  = { 0.0F, 0.0F, 0.0F };
    Vec3f attitude;
    float dt = 0.01F;

    // Test null context
    STATUS_TYPE status =
    FilterMadgwick6DOF ((FilterMadgwickContext*)0, &accel, &gyro, dt, &attitude);
    TEST_ASSERT_EQUAL_INT (eSTATUS_FAILURE, status);

    // Test null accel
    status = FilterMadgwick6DOF (&context, (Vec3f*)0, &gyro, dt, &attitude);
    TEST_ASSERT_EQUAL_INT (eSTATUS_FAILURE, status);

    // Test null gyro
    status = FilterMadgwick6DOF (&context, &accel, (Vec3f*)0, dt, &attitude);
    TEST_ASSERT_EQUAL_INT (eSTATUS_FAILURE, status);

    // Test null output
    status = FilterMadgwick6DOF (&context, &accel, &gyro, dt, (Vec3f*)0);
    TEST_ASSERT_EQUAL_INT (eSTATUS_FAILURE, status);
}

void test_FilterMadgwick6DOF_NoRotation (void) {
    FilterMadgwickContext context;
    FilterMadgwickInit (&context, 5.0F, NULL);

    // Stationary case: no gyro movement, gravity pointing down (Z-axis)
    Vec3f accel    = { 0.0F, 0.0F, 9.81F }; // Gravity in Z direction
    Vec3f gyro     = { 0.0F, 0.0F, 0.0F };  // No rotation
    Vec3f attitude = { 0.0F, 0.0F, 0.0F };
    float dt       = 0.01F;

    for (int i = 0; i < 5; i++) {
        STATUS_TYPE status =
        FilterMadgwick6DOF (&context, &accel, &gyro, dt, &attitude);
        TEST_ASSERT_EQUAL_INT (eSTATUS_SUCCESS, status);
    }

    // Should converge to no rotation (roll=0, pitch=0, yaw should be stable)
    TEST_ASSERT_FLOAT_WITHIN (0.1F, 0.0F, attitude.roll); // Roll should be ~0
    TEST_ASSERT_FLOAT_WITHIN (0.1F, 0.0F, attitude.pitch); // Pitch should be ~0
}

void test_FilterMadgwick6DOF_Roll90Degrees (void) {
    FilterMadgwickContext context;
    FilterMadgwickInit (&context, 0.1F, NULL);

    // Simulate 90-degree roll: gravity appears in Y direction
    Vec3f accel = { 0.0F, 9.81F, 0.0F }; // Gravity in Y direction (rolled 90 degrees)
    Vec3f gyro = { 10.0F, 0.0F, 0.0F }; // Rotating around the x-axis at 10 degrees/second
    Vec3f attitude = { 0.0F, 0.0F, 0.0F };
    float dt       = 0.01F;
    /*
     * Rotating at 10 degrees/second around the x - axis for 9 seconds puts
     * us close to 90 degrees. Especially with the 1.0F error.
     */
    for (int i = 0; i < (int)(9.0F * 1.0F / dt); i++) {
        STATUS_TYPE status =
        FilterMadgwick6DOF (&context, &accel, &gyro, dt, &attitude);
        TEST_ASSERT_EQUAL_INT (eSTATUS_SUCCESS, status);
    }

    // Should converge to 90-degree roll (or -90 depending on convention)
    TEST_ASSERT_FLOAT_WITHIN (2.0F, 90.0F, fabs (attitude.roll));
    TEST_ASSERT_FLOAT_WITHIN (0.01F, 0.0F, attitude.pitch);
    TEST_ASSERT_FLOAT_WITHIN (0.01F, 0.0F, attitude.yaw);
}

void test_FilterMadgwick6DOF_Pitch90Degrees (void) {
    FilterMadgwickContext context;
    FilterMadgwickInit (&context, 0.1F, NULL);

    // Simulate 90-degree pitch: gravity appears in X direction
    Vec3f accel = { 9.81F, 0.0F, 0.0F }; // Gravity in X direction (pitched 90 degrees)
    Vec3f gyro = { 0.0F, 10.0F, 0.0F }; // Rotating around the y-axis at 10 degrees/second
    Vec3f attitude = { 0.0F, 0.0F, 0.0F };
    float dt       = 0.01F;
    /*
     * Rotating at 10 degrees/second around the y - axis for 9 seconds puts
     * us close to 90 degrees. Especially with the 1.0F error.
     */
    for (int i = 0; i < (int)(9.0F * (1.0F / dt)); i++) {
        STATUS_TYPE status =
        FilterMadgwick6DOF (&context, &accel, &gyro, dt, &attitude);
        TEST_ASSERT_EQUAL_INT (eSTATUS_SUCCESS, status);
    }

    // Should converge to 90-degree pitch (or -90 depending on convention)
    TEST_ASSERT_FLOAT_WITHIN (0.01F, 0.0F, attitude.roll);
    TEST_ASSERT_FLOAT_WITHIN (2.0F, 90.0F, fabs (attitude.pitch));
    TEST_ASSERT_FLOAT_WITHIN (0.01F, 0.0F, attitude.yaw);
}

void test_FilterMadgwick6DOF_GyroIntegration (void) {
    FilterMadgwickContext context;
    FilterMadgwickInit (&context, 1.0F, NULL);

    Vec3f accel    = { 0.0F, 0.0F, 9.81F };
    Vec3f gyro     = { 0.0F, 0.0F, 45.0F }; // 45 degrees/second yaw rate
    Vec3f attitude = { 0.0F, 0.0F, 0.0F };
    float dt       = 0.01F;

    // Initialize with stable attitude first
    gyro.z = 0.0F;
    for (int i = 0; i < 50; i++) {
        FilterMadgwick6DOF (&context, &accel, &gyro, dt, &attitude);
    }
    float initial_yaw = attitude.yaw;

    // Now apply constant yaw rate for 1 second (100 iterations * 0.01s)
    gyro.z = 45.0F; // 45 degrees/second
    for (int i = 0; i < 100; i++) {
        STATUS_TYPE status =
        FilterMadgwick6DOF (&context, &accel, &gyro, dt, &attitude);
        TEST_ASSERT_EQUAL_INT (eSTATUS_SUCCESS, status);
    }

    // After 1 second at 45 deg/s, should have rotated ~45 degrees
    float yaw_change = attitude.yaw - initial_yaw;

    // Handle angle wrapping
    if (yaw_change > 180.0F) {
        yaw_change -= 360.0F;
    }
    if (yaw_change < -180.0F) {
        yaw_change += 360.0F;
    }

    TEST_ASSERT_FLOAT_WITHIN (15.0F, 45.0F, fabs (yaw_change));
}

void test_FilterMadgwick6DOF_ZeroAcceleration (void) {
    FilterMadgwickContext context;
    FilterMadgwickInit (&context, 5.0F, NULL);

    // Test with zero acceleration (should handle gracefully)
    Vec3f accel = { 0.0F, 0.0F, 0.0F };  // No acceleration
    Vec3f gyro  = { 10.0F, 0.0F, 0.0F }; // Some gyro input
    Vec3f attitude;
    float dt = 0.01F;

    STATUS_TYPE status = FilterMadgwick6DOF (&context, &accel, &gyro, dt, &attitude);
    TEST_ASSERT_EQUAL_INT (eSTATUS_SUCCESS, status);

    // Should not crash and should produce valid output
    TEST_ASSERT_TRUE (attitude.roll >= -180.0F && attitude.roll <= 180.0F);
    TEST_ASSERT_TRUE (attitude.pitch >= -90.0F && attitude.pitch <= 90.0F);
    TEST_ASSERT_TRUE (attitude.yaw >= -180.0F && attitude.yaw <= 180.0F);
}

void test_FilterMadgwick6DOF_QuaternionNormalization (void) {
    FilterMadgwickContext context;
    FilterMadgwickInit (&context, 5.0F, NULL);

    Vec3f accel    = { 0.0F, 0.0F, 9.81F };
    Vec3f gyro     = { 1.0F, 2.0F, 3.0F };
    Vec3f attitude = { 0.0F, 0.0F, 0.0F };
    float dt       = 0.01F;

    // Run several iterations
    for (int i = 0; i < 50; i++) {
        STATUS_TYPE status =
        FilterMadgwick6DOF (&context, &accel, &gyro, dt, &attitude);
        TEST_ASSERT_EQUAL_INT (eSTATUS_SUCCESS, status);

        // Check that quaternion remains normalized
        float quat_magnitude = sqrt (
        (context.est.q1 * context.est.q1) + (context.est.q2 * context.est.q2) +
        (context.est.q3 * context.est.q3) + (context.est.q4 * context.est.q4));
        TEST_ASSERT_FLOAT_WITHIN (0.01F, 1.0F, quat_magnitude);
    }
}

void test_FilterMadgwick6DOF_LargeTimeStep (void) {
    FilterMadgwickContext context;
    FilterMadgwickInit (&context, 0.01F, NULL);

    // Test with large time step and multi-axis rotation
    Vec3f accel = { 0.0F, 0.0F, 9.81F };
    Vec3f gyro = { 5.0F, 3.0F, 2.0F }; // Rotation rates: 5°/s roll, 3°/s pitch, 2°/s yaw
    Vec3f attitude = { 0.0F, 0.0F, 0.0F };
    float dt       = 0.02F; // 50 hz
    int iterations = 400;

    // Calculate expected total rotation for each axis
    float expected_roll_total =
    gyro.roll * dt * (float)iterations; // 5 * 1 * 10 = 50 degrees
    float expected_pitch_total =
    gyro.pitch * dt * (float)iterations; // 3 * 1 * 10 = 30 degrees
    float expected_yaw_total = gyro.yaw * dt * (float)iterations; // 2 * 1 * 10 = 20 degrees

    for (int i = 0; i < iterations; i++) {
        // Convert angles to radians for trigonometric functions
        float roll_rad  = (gyro.roll * dt * (float)i) * PI_F / 180.0F;
        float pitch_rad = (gyro.pitch * dt * (float)i) * PI_F / 180.0F;

        // Calculate gravity vector as seen by accelerometer based on
        // current attitude Gravity vector in body frame = R^T * [0,
        // 0, 9.81] Where R is rotation matrix from Euler angles
        accel.x = 9.81F * sinf (pitch_rad);
        accel.y = -9.81F * sinf (roll_rad) * cosf (pitch_rad);
        accel.z = 9.81F * cosf (roll_rad) * cosf (pitch_rad);

        // Verify magnitude is preserved (should always be 9.81)
        float accel_magnitude =
        sqrtf ((accel.x * accel.x) + (accel.y * accel.y) + (accel.z * accel.z));
        TEST_ASSERT_FLOAT_WITHIN (0.01F, 9.81F, accel_magnitude);

        STATUS_TYPE status =
        FilterMadgwick6DOF (&context, &accel, &gyro, dt, &attitude);
        TEST_ASSERT_EQUAL_INT (eSTATUS_SUCCESS, status);

        // Check that quaternion remains normalized even with large time steps
        float quat_magnitude = sqrtf (
        (context.est.q1 * context.est.q1) + (context.est.q2 * context.est.q2) +
        (context.est.q3 * context.est.q3) + (context.est.q4 * context.est.q4));
        TEST_ASSERT_FLOAT_WITHIN (0.01F, 1.0F, quat_magnitude);
    }

    float actual_roll_change  = attitude.roll;
    float actual_pitch_change = attitude.pitch;
    float actual_yaw_change   = attitude.yaw;

    // Test with reasonable tolerance since we're using large time steps
    TEST_ASSERT_FLOAT_WITHIN (4.0F, expected_roll_total, fabsf (actual_roll_change));
    TEST_ASSERT_FLOAT_WITHIN (4.0F, expected_pitch_total, fabsf (actual_pitch_change));
    TEST_ASSERT_FLOAT_WITHIN (10.0F, expected_yaw_total, fabsf (actual_yaw_change));
}

void test_FilterMadgwick6DOF_AccelNormalization (void) {
    FilterMadgwickContext context;
    FilterMadgwickInit (&context, 5.0F, NULL);

    // Test with different acceleration magnitudes (should be normalized internally)
    Vec3f accel1 = { 0.0F, 0.0F, 9.81F };  // Standard gravity
    Vec3f accel2 = { 0.0F, 0.0F, 19.62F }; // Double gravity
    Vec3f accel3 = { 0.0F, 0.0F, 4.905F }; // Half gravity
    Vec3f gyro   = { 0.0F, 0.0F, 0.0F };
    Vec3f attitude1;
    Vec3f attitude2;
    Vec3f attitude3;
    float dt = 0.01F;

    // Test with different acceleration magnitudes
    FilterMadgwickInit (&context, 5.0F, NULL);
    for (int i = 0; i < 100; i++) {
        FilterMadgwick6DOF (&context, &accel1, &gyro, dt, &attitude1);
    }

    FilterMadgwickInit (&context, 5.0F, NULL);
    for (int i = 0; i < 100; i++) {
        FilterMadgwick6DOF (&context, &accel2, &gyro, dt, &attitude2);
    }

    FilterMadgwickInit (&context, 5.0F, NULL);
    for (int i = 0; i < 100; i++) {
        FilterMadgwick6DOF (&context, &accel3, &gyro, dt, &attitude3);
    }

    // All should converge to similar attitudes since direction is the same
    TEST_ASSERT_FLOAT_WITHIN (5.0F, attitude1.roll, attitude2.roll);
    TEST_ASSERT_FLOAT_WITHIN (5.0F, attitude1.roll, attitude3.roll);
    TEST_ASSERT_FLOAT_WITHIN (5.0F, attitude1.pitch, attitude2.pitch);
    TEST_ASSERT_FLOAT_WITHIN (5.0F, attitude1.pitch, attitude3.pitch);
}
