
#include "test_runner.h"
// Manual test registration (for compilers without constructor support)
void register_all_tests (void) {
    // Forward declarations for all test functions
    // test_filter.c Tests
    extern void test_FilterMadgwickInit (void);
    extern void test_FilterMadgwick6DOF_NullPointers (void);
    extern void test_FilterMadgwick6DOF_NoRotation (void);
    extern void test_FilterMadgwick6DOF_Roll90Degrees (void);
    extern void test_FilterMadgwick6DOF_Pitch90Degrees (void);
    extern void test_FilterMadgwick6DOF_GyroIntegration (void);
    extern void test_FilterMadgwick6DOF_ZeroAcceleration (void);
    extern void test_FilterMadgwick6DOF_QuaternionNormalization (void);
    extern void test_FilterMadgwick6DOF_LargeTimeStep (void);
    extern void test_FilterMadgwick6DOF_AccelNormalization (void);

    // test_imu.c Tests
    extern void test_IMUInit (void);
    extern void test_IMUConf (void);
    extern void test_IMUUpdate (void);
    extern void test_IMUSelfCalibrate (void);

    // test_queue.c Tests
    extern void test_QueueInit_ValidParameters (void);
    extern void test_QueueInit_NullQueue (void);
    extern void test_QueueInit_NullBuffer (void);
    extern void test_QueueInit_ZeroCapacity (void);
    extern void test_QueueInit_NonPowerOf2Capacity (void);
    extern void test_QueueInit_ZeroElementSize (void);
    extern void test_QueueEnqueueDequeue_Basic (void);
    extern void test_QueueEnqueue_NullQueue (void);
    extern void test_QueueEnqueue_NullElement (void);
    extern void test_QueueDequeue_NullQueue (void);
    extern void test_QueueDequeue_NullElement (void);
    extern void test_QueueDequeue_EmptyQueue (void);
    extern void test_QueueFillToCapacity (void);
    extern void test_QueueEnqueue_FullQueue (void);
    extern void test_QueueFIFOOrdering (void);
    extern void test_QueueCircularBehavior (void);
    extern void test_QueuePeek_Basic (void);
    extern void test_QueuePeek_EmptyQueue (void);
    extern void test_QueuePeek_NullParameters (void);
    extern void test_QueueClear (void);
    extern void test_QueueClear_NullQueue (void);
    extern void test_QueueUint32 (void);
    extern void test_QueueStruct (void);
    extern void test_QueueUtilities_NullQueue (void);
    extern void test_QueueMultipleCycles (void);
    extern void test_QueuePeekMultiple (void);

    // Register all tests
    register_test ("test_FilterMadgwickInit", test_FilterMadgwickInit);
    register_test ("test_FilterMadgwick6DOF_NullPointers", test_FilterMadgwick6DOF_NullPointers);
    register_test ("test_FilterMadgwick6DOF_NoRotation", test_FilterMadgwick6DOF_NoRotation);
    register_test ("test_FilterMadgwick6DOF_Roll90Degrees", test_FilterMadgwick6DOF_Roll90Degrees);
    register_test ("test_FilterMadgwick6DOF_Pitch90Degrees", test_FilterMadgwick6DOF_Pitch90Degrees);
    register_test ("test_FilterMadgwick6DOF_GyroIntegration", test_FilterMadgwick6DOF_GyroIntegration);
    register_test ("test_FilterMadgwick6DOF_ZeroAcceleration", test_FilterMadgwick6DOF_ZeroAcceleration);
    register_test ("test_FilterMadgwick6DOF_QuaternionNormalization", test_FilterMadgwick6DOF_QuaternionNormalization);
    register_test ("test_FilterMadgwick6DOF_LargeTimeStep", test_FilterMadgwick6DOF_LargeTimeStep);
    register_test ("test_FilterMadgwick6DOF_AccelNormalization", test_FilterMadgwick6DOF_AccelNormalization);
    register_test ("test_IMUInit", test_IMUInit);
    register_test ("test_IMUConf", test_IMUConf);
    register_test ("test_IMUUpdate", test_IMUUpdate);
    register_test ("test_IMUSelfCalibrate", test_IMUSelfCalibrate);
    register_test ("test_QueueInit_ValidParameters", test_QueueInit_ValidParameters);
    register_test ("test_QueueInit_NullQueue", test_QueueInit_NullQueue);
    register_test ("test_QueueInit_NullBuffer", test_QueueInit_NullBuffer);
    register_test ("test_QueueInit_ZeroCapacity", test_QueueInit_ZeroCapacity);
    register_test ("test_QueueInit_NonPowerOf2Capacity", test_QueueInit_NonPowerOf2Capacity);
    register_test ("test_QueueInit_ZeroElementSize", test_QueueInit_ZeroElementSize);
    register_test ("test_QueueEnqueueDequeue_Basic", test_QueueEnqueueDequeue_Basic);
    register_test ("test_QueueEnqueue_NullQueue", test_QueueEnqueue_NullQueue);
    register_test ("test_QueueEnqueue_NullElement", test_QueueEnqueue_NullElement);
    register_test ("test_QueueDequeue_NullQueue", test_QueueDequeue_NullQueue);
    register_test ("test_QueueDequeue_NullElement", test_QueueDequeue_NullElement);
    register_test ("test_QueueDequeue_EmptyQueue", test_QueueDequeue_EmptyQueue);
    register_test ("test_QueueFillToCapacity", test_QueueFillToCapacity);
    register_test ("test_QueueEnqueue_FullQueue", test_QueueEnqueue_FullQueue);
    register_test ("test_QueueFIFOOrdering", test_QueueFIFOOrdering);
    register_test ("test_QueueCircularBehavior", test_QueueCircularBehavior);
    register_test ("test_QueuePeek_Basic", test_QueuePeek_Basic);
    register_test ("test_QueuePeek_EmptyQueue", test_QueuePeek_EmptyQueue);
    register_test ("test_QueuePeek_NullParameters", test_QueuePeek_NullParameters);
    register_test ("test_QueueClear", test_QueueClear);
    register_test ("test_QueueClear_NullQueue", test_QueueClear_NullQueue);
    register_test ("test_QueueUint32", test_QueueUint32);
    register_test ("test_QueueStruct", test_QueueStruct);
    register_test ("test_QueueUtilities_NullQueue", test_QueueUtilities_NullQueue);
    register_test ("test_QueueMultipleCycles", test_QueueMultipleCycles);
    register_test ("test_QueuePeekMultiple", test_QueuePeekMultiple);
}
