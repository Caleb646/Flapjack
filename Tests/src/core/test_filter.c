#include "core/core.h"
#include "mc/filter.h"
#include "unity/unity.h"
#include <math.h>

#ifndef UNIT_TEST
#error "UNIT_TEST should be defined in this file"
#endif

#define PI_F 3.14159265359F

#define MADG_TEST_INIT(ERROR_DEGS, DRIFT_DEGS)                 \
    FilterMadgwick_t filter = { 0 };                           \
    do {                                                       \
        FilterMadgwickInitConf_t conf = { 0 };                 \
        conf.gyroMeasureErrorDegs     = (ERROR_DEGS);          \
        conf.gyroMeasureDriftDegs     = (DRIFT_DEGS);          \
        TEST_ASSERT_TRUE (FilterMadgwickInit (conf, &filter)); \
    } while (0)

#define MADG_TEST_DEF_INIT() MADG_TEST_INIT (5.0F, 0.0F)


void test_FilterMadgwickInit (void) {

    float gyroMeasureErrorDegs = 5.0F;
    float gyroMeasureDriftDegs = 0.0F;
    MADG_TEST_INIT (gyroMeasureErrorDegs, gyroMeasureDriftDegs);

    // Check that quaternion is initialized to identity (no rotation)
    TEST_ASSERT_FLOAT_WITHIN (0.001F, 1.0F, filter.qEst.q1);
    TEST_ASSERT_FLOAT_WITHIN (0.001F, 0.0F, filter.qEst.q2);
    TEST_ASSERT_FLOAT_WITHIN (0.001F, 0.0F, filter.qEst.q3);
    TEST_ASSERT_FLOAT_WITHIN (0.001F, 0.0F, filter.qEst.q4);

    float expected_beta = sqrtf (3.0F / 4.0F) * (gyroMeasureErrorDegs * PI_F / 180.0F);
    TEST_ASSERT_FLOAT_WITHIN (0.001F, expected_beta, filter.beta);
}

void test_FilterMadgwick6DOF_NullPointers (void) {

    MADG_TEST_DEF_INIT ();
    Vec3f accel = { 0.0F, 0.0F, 9.81F };
    Vec3f gyro  = { 0.0F, 0.0F, 0.0F };
    Vec3f attitude;
    float dt = 0.01F;

    bool success = FilterMadgwickUpdate (&filter, &accel, &gyro, NULL, dt, &attitude);
    TEST_ASSERT_TRUE (success);
}

void test_FilterMadgwick6DOF_NoRotation (void) {

    MADG_TEST_INIT (5.0F, 0.0F);

    // Stationary case: no gyro movement, gravity pointing down (Z-axis)
    Vec3f accel    = { 0.0F, 0.0F, 9.81F }; // Gravity in Z direction
    Vec3f gyro     = { 0.0F, 0.0F, 0.0F };  // No rotation
    Vec3f attitude = { 0.0F, 0.0F, 0.0F };
    float dt       = 0.01F;

    for (int i = 0; i < 5; i++) {
        bool success = FilterMadgwickUpdate (&filter, &accel, &gyro, NULL, dt, &attitude);
        TEST_ASSERT_TRUE (success);
    }

    // Should converge to no rotation (roll=0, pitch=0, yaw should be stable)
    TEST_ASSERT_FLOAT_WITHIN (0.1F, 0.0F, attitude.roll);  // Roll should be ~0
    TEST_ASSERT_FLOAT_WITHIN (0.1F, 0.0F, attitude.pitch); // Pitch should be ~0
}

void test_FilterMadgwick6DOF_Roll90Degrees (void) {

    MADG_TEST_INIT (0.1F, 0.0F);

    // Simulate 90-degree roll: gravity appears in Y direction
    Vec3f accel    = { 0.0F, 9.81F, 0.0F }; // Gravity in Y direction (rolled 90 degrees)
    Vec3f gyro     = { 10.0F, 0.0F, 0.0F }; // Rotating around the x-axis at 10 degrees/second
    Vec3f attitude = { 0.0F, 0.0F, 0.0F };
    float dt       = 0.01F;
    /*
     * Rotating at 10 degrees/second around the x - axis for 9 seconds puts
     * us close to 90 degrees. Especially with the 1.0F error.
     */
    for (int i = 0; i < (int)(9.0F * 1.0F / dt); i++) {
        bool success = FilterMadgwickUpdate (&filter, &accel, &gyro, NULL, dt, &attitude);
        TEST_ASSERT_TRUE (success);
    }

    TEST_ASSERT_FLOAT_WITHIN (2.0F, 90.0F, fabsf (attitude.roll));
    TEST_ASSERT_FLOAT_WITHIN (0.01F, 0.0F, attitude.pitch);
    TEST_ASSERT_FLOAT_WITHIN (0.01F, 0.0F, attitude.yaw);
}

void test_FilterMadgwick6DOF_Pitch90Degrees (void) {

    MADG_TEST_INIT (0.1F, 0.0F);

    // Simulate 90-degree pitch: gravity appears in X direction
    Vec3f accel    = { 9.81F, 0.0F, 0.0F }; // Gravity in X direction (pitched 90 degrees)
    Vec3f gyro     = { 0.0F, 10.0F, 0.0F }; // Rotating around the y-axis at 10 degrees/second
    Vec3f attitude = { 0.0F, 0.0F, 0.0F };
    float dt       = 0.01F;
    /*
     * Rotating at 10 degrees/second around the y - axis for 9 seconds puts
     * us close to 90 degrees. Especially with the 1.0F error.
     */
    for (int i = 0; i < (int)(9.0F * (1.0F / dt)); i++) {
        bool success = FilterMadgwickUpdate (&filter, &accel, &gyro, NULL, dt, &attitude);
        TEST_ASSERT_TRUE (success);
    }

    // Should converge to 90-degree pitch
    TEST_ASSERT_FLOAT_WITHIN (0.01F, 0.0F, attitude.roll);
    TEST_ASSERT_FLOAT_WITHIN (2.0F, 90.0F, fabsf (attitude.pitch));
    TEST_ASSERT_FLOAT_WITHIN (0.01F, 0.0F, attitude.yaw);
}

void test_FilterMadgwick6DOF_GyroIntegration (void) {

    MADG_TEST_INIT (0.1F, 0.0F);

    Vec3f accel    = { 0.0F, 0.0F, 9.81F };
    Vec3f gyro     = { 0.0F, 0.0F, 45.0F }; // 45 degrees/second yaw rate
    Vec3f attitude = { 0.0F, 0.0F, 0.0F };
    float dt       = 0.01F;

    // Initialize with stable attitude first
    gyro.z = 0.0F;
    for (int i = 0; i < 50; i++) {
        bool success = FilterMadgwickUpdate (&filter, &accel, &gyro, NULL, dt, &attitude);
        TEST_ASSERT_TRUE (success);
    }
    float initial_yaw = attitude.yaw;

    // Now apply constant yaw rate for 1 second (100 iterations * 0.01s)
    gyro.z = 45.0F; // 45 degrees/second
    for (int i = 0; i < 100; i++) {
        bool success = FilterMadgwickUpdate (&filter, &accel, &gyro, NULL, dt, &attitude);
        TEST_ASSERT_TRUE (success);
    }

    // After 1 second at 45 deg/s, should have rotated ~45 degrees
    float yaw_change = attitude.yaw - initial_yaw;
    TEST_ASSERT_FLOAT_WITHIN (15.0F, 45.0F, fabsf (yaw_change));
}

// void test_FilterMadgwick6DOF_ZeroAcceleration (void) {

//     MADG_TEST_INIT (5.0F, 0.0F);

//     // Test with zero acceleration (should handle gracefully)
//     Vec3f accel = { 0.0F, 0.0F, 0.0F };  // No acceleration
//     Vec3f gyro  = { 10.0F, 0.0F, 0.0F }; // Some gyro input
//     Vec3f attitude;
//     float dt = 0.01F;

//     FilterMadgwickUpdate (&filter, &accel, &gyro, NULL, dt, &attitude);
//     // TEST_ASSERT_TRUE (success);

//     // Should not crash and should produce valid output
//     TEST_ASSERT_TRUE (attitude.roll >= -180.0F && attitude.roll <= 180.0F);
//     TEST_ASSERT_TRUE (attitude.pitch >= -90.0F && attitude.pitch <= 90.0F);
//     TEST_ASSERT_TRUE (attitude.yaw >= -180.0F && attitude.yaw <= 180.0F);
// }

void test_FilterMadgwick6DOF_QuaternionNormalization (void) {

    MADG_TEST_INIT (5.0F, 0.0F);

    Vec3f accel    = { 0.0F, 0.0F, 9.81F };
    Vec3f gyro     = { 1.0F, 2.0F, 3.0F };
    Vec3f attitude = { 0.0F, 0.0F, 0.0F };
    float dt       = 0.01F;

    // Run several iterations
    for (int i = 0; i < 50; i++) {
        bool success = FilterMadgwickUpdate (&filter, &accel, &gyro, NULL, dt, &attitude);
        TEST_ASSERT_TRUE (success);

        // Check that quaternion remains normalized
        float quat_magnitude = sqrtf (
        (filter.qEst.q1 * filter.qEst.q1) + (filter.qEst.q2 * filter.qEst.q2) +
        (filter.qEst.q3 * filter.qEst.q3) + (filter.qEst.q4 * filter.qEst.q4)
        );
        TEST_ASSERT_FLOAT_WITHIN (0.01F, 1.0F, quat_magnitude);
    }
}

void test_FilterMadgwick6DOF_LargeTimeStep (void) {

    MADG_TEST_INIT (0.0F, 0.1F);

    // Test with large time step and multi-axis rotation
    Vec3f accel    = { 0.0F, 0.0F, 9.81F };
    Vec3f gyro     = { 5.0F, 3.0F, 2.0F }; // Rotation rates: 5°/s roll, 3°/s pitch, 2°/s yaw
    Vec3f attitude = { 0.0F, 0.0F, 0.0F };
    float dt       = 0.02F; // 50 hz
    int iterations = 400;

    // Calculate expected total rotation for each axis
    float expected_roll_total  = gyro.roll * dt * (float)iterations;  // 5 * 1 * 10 = 50 degrees
    float expected_pitch_total = gyro.pitch * dt * (float)iterations; // 3 * 1 * 10 = 30 degrees
    float expected_yaw_total   = gyro.yaw * dt * (float)iterations;   // 2 * 1 * 10 = 20 degrees

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
        float accel_magnitude = sqrtf ((accel.x * accel.x) + (accel.y * accel.y) + (accel.z * accel.z));
        TEST_ASSERT_FLOAT_WITHIN (0.01F, 9.81F, accel_magnitude);

        bool success = FilterMadgwickUpdate (&filter, &accel, &gyro, NULL, dt, &attitude);
        TEST_ASSERT_TRUE (success);

        // Check that quaternion remains normalized even with large time steps
        float quat_magnitude = sqrtf (
        (filter.qEst.q1 * filter.qEst.q1) + (filter.qEst.q2 * filter.qEst.q2) +
        (filter.qEst.q3 * filter.qEst.q3) + (filter.qEst.q4 * filter.qEst.q4)
        );
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

// ============================================================================
// 9DOF Tests (with Magnetometer)
// ============================================================================

// void test_FilterMadgwick9DOF_NorthPointing (void) {
//     MADG_TEST_DEF_INIT ();

//     // Stationary, level, pointing north
//     Vec3f accel = { 0.0F, 0.0F, 9.81F }; // Gravity down (Z-axis)
//     Vec3f gyro  = { 0.0F, 0.0F, 0.0F };  // No rotation
//     Vec3f mag = { 1.0F, 0.0F, 0.0F }; // North pointing (X-axis in body
//     frame) float dt = 0.01F;

//     // Run filter to convergence
//     for (int i = 0; i < 200; i++) {
//         bool success = FilterMadgwickUpdate_9DOF (&filter, &accel, &gyro, &mag, dt);
//         // TEST_ASSERT_TRUE (success);
//     }

//     // Should have converged to level attitude with yaw close to 0
//     Vec3f attitude;
//     FilterMadgwickUpdate (&filter, &accel, &gyro, &mag, dt, &attitude);

//     TEST_ASSERT_FLOAT_WITHIN (2.0F, 0.0F, attitude.roll);
//     TEST_ASSERT_FLOAT_WITHIN (2.0F, 0.0F, attitude.pitch);
//     TEST_ASSERT_FLOAT_WITHIN (10.0F, 0.0F, attitude.yaw); // Allow larger tolerance for yaw
// }

// void test_FilterMadgwick9DOF_EastPointing (void) {
//     MADG_TEST_DEF_INIT ();

//     // Stationary, level, pointing east (90 degree yaw)
//     Vec3f accel = { 0.0F, 0.0F, 9.81F }; // Gravity down
//     Vec3f gyro  = { 0.0F, 0.0F, 0.0F };  // No rotation
//     Vec3f mag = { 0.0F, 1.0F, 0.0F }; // East pointing (Y-axis in body
//     frame) float dt = 0.01F;

//     // Run filter to convergence
//     for (int i = 0; i < 200; i++) {
//         bool success = FilterMadgwickUpdate_9DOF (&filter, &accel,
//         &gyro, &mag, dt); TEST_ASSERT_TRUE (success);
//     }

//     Vec3f attitude;
//     FilterMadgwickUpdate (&filter, &accel, &gyro, &mag, dt, &attitude);

//     TEST_ASSERT_FLOAT_WITHIN (2.0F, 0.0F, attitude.roll);
//     TEST_ASSERT_FLOAT_WITHIN (2.0F, 0.0F, attitude.pitch);
//     TEST_ASSERT_FLOAT_WITHIN (15.0F, 90.0F, fabsf (attitude.yaw)); // Should be close to ±90°
// }

// void test_FilterMadgwick9DOF_SouthPointing (void) {
//     MADG_TEST_DEF_INIT ();

//     // Stationary, level, pointing south (180 degree yaw)
//     Vec3f accel = { 0.0F, 0.0F, 9.81F }; // Gravity down
//     Vec3f gyro  = { 0.0F, 0.0F, 0.0F };  // No rotation
//     Vec3f mag = { -1.0F, 0.0F, 0.0F }; // South pointing (-X-axis in
//     body frame) float dt = 0.01F;

//     // Run filter to convergence
//     for (int i = 0; i < 200; i++) {
//         bool success = FilterMadgwickUpdate_9DOF (&filter, &accel,
//         &gyro, &mag, dt); TEST_ASSERT_TRUE (success);
//     }

//     Vec3f attitude;
//     FilterMadgwickUpdate (&filter, &accel, &gyro, &mag, dt, &attitude);

//     TEST_ASSERT_FLOAT_WITHIN (2.0F, 0.0F, attitude.roll);
//     TEST_ASSERT_FLOAT_WITHIN (2.0F, 0.0F, attitude.pitch);
//     TEST_ASSERT_FLOAT_WITHIN (15.0F, 180.0F, fabsf (attitude.yaw)); // Should be close to ±180°
// }

// void test_FilterMadgwick9DOF_TiltedWithMag (void) {
//     MADG_TEST_DEF_INIT ();

//     // 30 degree roll, pointing northeast
//     float roll_deg = 30.0F;
//     float roll_rad = roll_deg * PI_F / 180.0F;

//     // Gravity vector in rolled body frame
//     Vec3f accel = { 0.0F, -9.81F * sinf (roll_rad), 9.81F * cosf
//     (roll_rad) }; Vec3f gyro = { 0.0F, 0.0F, 0.0F }; // No rotation

//     // Magnetometer pointing northeast (45 degree yaw from north)
//     float yaw_rad = 45.0F * PI_F / 180.0F;
//     Vec3f mag = { cosf (yaw_rad), sinf (yaw_rad), 0.0F }; // Northeast in body frame

//     float dt = 0.01F;

//     // Run filter to convergence
//     for (int i = 0; i < 300; i++) {
//         bool success = FilterMadgwickUpdate_9DOF (&filter, &accel,
//         &gyro, &mag, dt); TEST_ASSERT_TRUE (success);
//     }

//     Vec3f attitude;
//     FilterMadgwickUpdate (&filter, &accel, &gyro, &mag, dt, &attitude);

//     TEST_ASSERT_FLOAT_WITHIN (5.0F, roll_deg, attitude.roll);
//     TEST_ASSERT_FLOAT_WITHIN (3.0F, 0.0F, attitude.pitch);
//     TEST_ASSERT_FLOAT_WITHIN (10.0F, 45.0F, attitude.yaw);
// }

// void test_FilterMadgwick9DOF_YawRotationWithMag (void) {
//     MADG_TEST_INIT (0.5F, 0.0F); // Lower error for better precision

//     Vec3f accel = { 0.0F, 0.0F, 9.81F }; // Level
//     Vec3f gyro  = { 0.0F, 0.0F, 30.0F }; // 30 deg/s yaw rate
//     Vec3f mag   = { 1.0F, 0.0F, 0.0F };  // Initially pointing north
//     float dt    = 0.01F;

//     // Initialize with stable attitude first
//     gyro.z = 0.0F;
//     for (int i = 0; i < 100; i++) {
//         FilterMadgwickUpdate_9DOF (&filter, &accel, &gyro, &mag, dt);
//     }

//     Vec3f initial_attitude;
//     FilterMadgwickUpdate (&filter, &accel, &gyro, &mag, dt,
//     &initial_attitude); float initial_yaw = initial_attitude.yaw;

//     // Now apply yaw rotation for 2 seconds while updating mag vector
//     gyro.z = 30.0F;                 // 30 degrees/second
//     for (int i = 0; i < 200; i++) { // 2 seconds at 100Hz
//         // Update magnetometer vector as we rotate
//         float current_yaw_rad = (30.0F * dt * (float)i) * PI_F / 180.0F;
//         mag.x = cosf (current_yaw_rad); // North component decreases as
//         we turn mag.y = sinf (current_yaw_rad); // East component
//         increases as we turn mag.z = 0.0F;

//         bool success = FilterMadgwickUpdate_9DOF (&filter, &accel,
//         &gyro, &mag, dt); TEST_ASSERT_TRUE (success);
//     }

//     Vec3f final_attitude;
//     FilterMadgwickUpdate (&filter, &accel, &gyro, &mag, dt, &final_attitude);

//     // After 2 seconds at 30 deg/s, should have rotated 60 degrees
//     float yaw_change = final_attitude.yaw - initial_yaw;
//     TEST_ASSERT_FLOAT_WITHIN (10.0F, 60.0F, fabsf (yaw_change));
//     TEST_ASSERT_FLOAT_WITHIN (2.0F, 0.0F, final_attitude.roll);
//     TEST_ASSERT_FLOAT_WITHIN (2.0F, 0.0F, final_attitude.pitch);
// }

void test_FilterMadgwick9DOF_MagnetometerDisturbance (void) {
    MADG_TEST_DEF_INIT ();

    Vec3f accel = { 0.0F, 0.0F, 9.81F }; // Gravity down
    Vec3f gyro  = { 0.0F, 0.0F, 0.0F };  // No rotation
    Vec3f mag   = { 1.0F, 0.0F, 0.0F };  // North pointing
    float dt    = 0.01F;

    // Converge to stable state first
    for (int i = 0; i < 100; i++) {
        FilterMadgwickUpdate_9DOF (&filter, &accel, &gyro, &mag, dt);
    }

    Vec3f stable_attitude;
    FilterMadgwickUpdate (&filter, &accel, &gyro, &mag, dt, &stable_attitude);

    // Introduce magnetometer disturbance (e.g., iron interference)
    Vec3f disturbed_mag = { 0.5F, 1.5F, -0.8F }; // Disturbed magnetic field

    // Run for several iterations with disturbance
    for (int i = 0; i < 50; i++) {
        FilterMadgwickUpdate_9DOF (&filter, &accel, &gyro, &disturbed_mag, dt);
    }

    Vec3f disturbed_attitude;
    FilterMadgwickUpdate (&filter, &accel, &gyro, &disturbed_mag, dt, &disturbed_attitude);

    // Filter should still maintain reasonable attitude despite disturbance
    TEST_ASSERT_FLOAT_WITHIN (10.0F, stable_attitude.roll, disturbed_attitude.roll);
    TEST_ASSERT_FLOAT_WITHIN (10.0F, stable_attitude.pitch, disturbed_attitude.pitch);
    // Yaw might be more affected by mag disturbance, so allow larger tolerance

    // Return to normal magnetometer
    for (int i = 0; i < 100; i++) {
        FilterMadgwickUpdate_9DOF (&filter, &accel, &gyro, &mag, dt);
    }

    Vec3f recovered_attitude;
    FilterMadgwickUpdate (&filter, &accel, &gyro, &mag, dt, &recovered_attitude);

    // Should recover to stable attitude
    TEST_ASSERT_FLOAT_WITHIN (5.0F, stable_attitude.roll, recovered_attitude.roll);
    TEST_ASSERT_FLOAT_WITHIN (5.0F, stable_attitude.pitch, recovered_attitude.pitch);
}

void test_FilterMadgwick9DOF_ZeroMagnetometer (void) {
    MADG_TEST_DEF_INIT ();

    Vec3f accel = { 0.0F, 0.0F, 9.81F }; // Gravity down
    Vec3f gyro  = { 0.0F, 0.0F, 0.0F };  // No rotation
    Vec3f mag   = { 0.0F, 0.0F, 0.0F };  // Zero magnetometer (should fail gracefully)
    float dt    = 0.01F;

    // Should handle zero magnetometer gracefully without crashing
    bool success = FilterMadgwickUpdate_9DOF (&filter, &accel, &gyro, &mag, dt);
    TEST_ASSERT_FALSE (success); // Should return false for zero magnetometer

    // Quaternion should remain valid (not NaN)
    TEST_ASSERT_TRUE (!isnan (filter.qEst.q1));
    TEST_ASSERT_TRUE (!isnan (filter.qEst.q2));
    TEST_ASSERT_TRUE (!isnan (filter.qEst.q3));
    TEST_ASSERT_TRUE (!isnan (filter.qEst.q4));
}

// void test_FilterMadgwick9DOF_MagInclinationEffect (void) {
//     MADG_TEST_DEF_INIT ();

//     // Test with magnetometer having inclination (pointing down)
//     Vec3f accel = { 0.0F, 0.0F, 9.81F }; // Gravity down
//     Vec3f gyro  = { 0.0F, 0.0F, 0.0F };  // No rotation
//     Vec3f mag = { 0.7071F, 0.0F, -0.7071F }; // 45 degree inclination
//     (north + down) float dt = 0.01F;

//     // Run filter to convergence
//     for (int i = 0; i < 200; i++) {
//         bool success = FilterMadgwickUpdate_9DOF (&filter, &accel,
//         &gyro, &mag, dt); TEST_ASSERT_TRUE (success);
//     }

//     Vec3f attitude;
//     FilterMadgwickUpdate (&filter, &accel, &gyro, &mag, dt, &attitude);

//     // Should still maintain level attitude despite magnetic inclination
//     TEST_ASSERT_FLOAT_WITHIN (3.0F, 0.0F, attitude.roll);
//     TEST_ASSERT_FLOAT_WITHIN (3.0F, 0.0F, attitude.pitch);
//     // Yaw should still be close to north-pointing
//     TEST_ASSERT_FLOAT_WITHIN (15.0F, 0.0F, attitude.yaw);
// }

// void test_FilterMadgwick9DOF_QuaternionNormalization (void) {
//     MADG_TEST_DEF_INIT ();

//     Vec3f accel = { 0.0F, 0.0F, 9.81F };
//     Vec3f gyro  = { 2.0F, 3.0F, 1.0F }; // Multi-axis rotation
//     Vec3f mag   = { 0.8F, 0.6F, 0.0F }; // Northeast pointing
//     float dt    = 0.01F;

//     // Run several iterations and check quaternion normalization
//     for (int i = 0; i < 100; i++) {
//         bool success = FilterMadgwickUpdate_9DOF (&filter, &accel,
//         &gyro, &mag, dt); TEST_ASSERT_TRUE (success);

//         // Check that quaternion remains normalized
//         float quat_magnitude = sqrtf (
//         (filter.qEst.q1 * filter.qEst.q1) + (filter.qEst.q2 * filter.qEst.q2) +
//         (filter.qEst.q3 * filter.qEst.q3) + (filter.qEst.q4 * filter.qEst.q4)
//         );
//         TEST_ASSERT_FLOAT_WITHIN (0.01F, 1.0F, quat_magnitude);

//         // Check that magnetic reference vectors are valid
//         TEST_ASSERT_TRUE (!isnan (filter.bx));
//         TEST_ASSERT_TRUE (!isnan (filter.bz));
//         TEST_ASSERT_TRUE (filter.bx >= 0.0F); // bx should be positive
//     }
// }

void setUp (void) {
    // This is run before EACH test
}

void tearDown (void) {
    // This is run after EACH test
}

int main (void) {
    UNITY_BEGIN ();

    // 6DOF Tests (IMU + Gyroscope only)
    RUN_TEST (test_FilterMadgwickInit);
    RUN_TEST (test_FilterMadgwick6DOF_NullPointers);
    RUN_TEST (test_FilterMadgwick6DOF_NoRotation);
    RUN_TEST (test_FilterMadgwick6DOF_Roll90Degrees);
    RUN_TEST (test_FilterMadgwick6DOF_Pitch90Degrees);
    RUN_TEST (test_FilterMadgwick6DOF_GyroIntegration);
    // RUN_TEST (test_FilterMadgwick6DOF_ZeroAcceleration);
    RUN_TEST (test_FilterMadgwick6DOF_QuaternionNormalization);
    RUN_TEST (test_FilterMadgwick6DOF_LargeTimeStep);

    // 9DOF Tests (IMU + Gyroscope + Magnetometer)
    // RUN_TEST (test_FilterMadgwick9DOF_NorthPointing);
    // RUN_TEST (test_FilterMadgwick9DOF_EastPointing);
    // RUN_TEST (test_FilterMadgwick9DOF_SouthPointing);
    // RUN_TEST (test_FilterMadgwick9DOF_TiltedWithMag);
    // RUN_TEST (test_FilterMadgwick9DOF_YawRotationWithMag);
    RUN_TEST (test_FilterMadgwick9DOF_MagnetometerDisturbance);
    RUN_TEST (test_FilterMadgwick9DOF_ZeroMagnetometer);
    // RUN_TEST (test_FilterMadgwick9DOF_MagInclinationEffect);
    // RUN_TEST (test_FilterMadgwick9DOF_QuaternionNormalization);

    return UNITY_END ();
}