#include "unity/unity.h"

#ifndef UNIT_TEST
#error "UNIT_TEST should be defined in this file"
#endif

void setUp (void) {
}

void tearDown (void) {
}

// Forward declarations for test functions from test_imu.c
void test_IMUInit (void);
void test_IMUConf (void);
void test_IMUUpdate (void);

// Forward declarations for test functions from test_filter.c
void test_FilterMadgwickInit (void);
void test_FilterMadgwick6DOF_NullPointers (void);
void test_FilterMadgwick6DOF_NoRotation (void);
void test_FilterMadgwick6DOF_Roll90Degrees (void);
void test_FilterMadgwick6DOF_Pitch90Degrees (void);
void test_FilterMadgwick6DOF_GyroIntegration (void);
void test_FilterMadgwick6DOF_ZeroAcceleration (void);
void test_FilterMadgwick6DOF_QuaternionNormalization (void);
void test_FilterMadgwick6DOF_LargeTimeStep (void);
void test_FilterMadgwick6DOF_AccelNormalization (void);

int main (void) {
    UNITY_BEGIN ();

    // IMU Tests
    RUN_TEST (test_IMUInit);
    RUN_TEST (test_IMUConf);
    RUN_TEST (test_IMUUpdate);

    // Filter Tests
    RUN_TEST (test_FilterMadgwickInit);
    RUN_TEST (test_FilterMadgwick6DOF_NullPointers);
    RUN_TEST (test_FilterMadgwick6DOF_NoRotation);
    RUN_TEST (test_FilterMadgwick6DOF_Roll90Degrees);
    RUN_TEST (test_FilterMadgwick6DOF_Pitch90Degrees);
    RUN_TEST (test_FilterMadgwick6DOF_GyroIntegration);
    RUN_TEST (test_FilterMadgwick6DOF_ZeroAcceleration);
    RUN_TEST (test_FilterMadgwick6DOF_QuaternionNormalization);
    RUN_TEST (test_FilterMadgwick6DOF_LargeTimeStep);
    RUN_TEST (test_FilterMadgwick6DOF_AccelNormalization);

    return UNITY_END ();
}
