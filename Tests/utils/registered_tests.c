
#include "test_runner.h"
void RegisterAllTests(void) {
    // test_filter.c Tests
    extern void test_FilterMadgwickInit(void);
    extern void test_FilterMadgwick6DOF_NullPointers(void);
    extern void test_FilterMadgwick6DOF_NoRotation(void);
    extern void test_FilterMadgwick6DOF_Roll90Degrees(void);
    extern void test_FilterMadgwick6DOF_Pitch90Degrees(void);
    extern void test_FilterMadgwick6DOF_GyroIntegration(void);
    extern void test_FilterMadgwick6DOF_ZeroAcceleration(void);
    extern void test_FilterMadgwick6DOF_QuaternionNormalization(void);
    extern void test_FilterMadgwick6DOF_LargeTimeStep(void);

    // test_imu.c Tests
    extern void test_IMUInit(void);
    extern void test_IMUConf(void);
    extern void test_IMUUpdate(void);
    extern void test_IMUSelfCalibrate(void);

    // test_queue.c Tests
    extern void test_QueueInit_ValidParameters(void);
    extern void test_QueueInit_NullQueue(void);
    extern void test_QueueInit_NullBuffer(void);
    extern void test_QueueInit_ZeroCapacity(void);
    extern void test_QueueInit_NonPowerOf2Capacity(void);
    extern void test_QueueInit_ZeroElementSize(void);
    extern void test_QueueEnqueueDequeue_Basic(void);
    extern void test_QueueEnqueue_NullQueue(void);
    extern void test_QueueEnqueue_NullElement(void);
    extern void test_QueueDequeue_NullQueue(void);
    extern void test_QueueDequeue_NullElement(void);
    extern void test_QueueDequeue_EmptyQueue(void);
    extern void test_QueueFillToCapacity(void);
    extern void test_QueueEnqueue_FullQueue(void);
    extern void test_QueueFIFOOrdering(void);
    extern void test_QueueCircularBehavior(void);
    extern void test_QueuePeek_Basic(void);
    extern void test_QueuePeek_EmptyQueue(void);
    extern void test_QueuePeek_NullParameters(void);
    extern void test_QueueClear(void);
    extern void test_QueueUint32(void);
    extern void test_QueueStruct(void);
    extern void test_QueueUtilities_NullQueue(void);
    extern void test_QueueMultipleCycles(void);
    extern void test_QueuePeekMultiple(void);
    
    RegisterTest("test_FilterMadgwickInit", test_FilterMadgwickInit);
    RegisterTest("test_FilterMadgwick6DOF_NullPointers", test_FilterMadgwick6DOF_NullPointers);
    RegisterTest("test_FilterMadgwick6DOF_NoRotation", test_FilterMadgwick6DOF_NoRotation);
    RegisterTest("test_FilterMadgwick6DOF_Roll90Degrees", test_FilterMadgwick6DOF_Roll90Degrees);
    RegisterTest("test_FilterMadgwick6DOF_Pitch90Degrees", test_FilterMadgwick6DOF_Pitch90Degrees);
    RegisterTest("test_FilterMadgwick6DOF_GyroIntegration", test_FilterMadgwick6DOF_GyroIntegration);
    RegisterTest("test_FilterMadgwick6DOF_ZeroAcceleration", test_FilterMadgwick6DOF_ZeroAcceleration);
    RegisterTest("test_FilterMadgwick6DOF_QuaternionNormalization", test_FilterMadgwick6DOF_QuaternionNormalization);
    RegisterTest("test_FilterMadgwick6DOF_LargeTimeStep", test_FilterMadgwick6DOF_LargeTimeStep);
    RegisterTest("test_IMUInit", test_IMUInit);
    RegisterTest("test_IMUConf", test_IMUConf);
    RegisterTest("test_IMUUpdate", test_IMUUpdate);
    RegisterTest("test_IMUSelfCalibrate", test_IMUSelfCalibrate);
    RegisterTest("test_QueueInit_ValidParameters", test_QueueInit_ValidParameters);
    RegisterTest("test_QueueInit_NullQueue", test_QueueInit_NullQueue);
    RegisterTest("test_QueueInit_NullBuffer", test_QueueInit_NullBuffer);
    RegisterTest("test_QueueInit_ZeroCapacity", test_QueueInit_ZeroCapacity);
    RegisterTest("test_QueueInit_NonPowerOf2Capacity", test_QueueInit_NonPowerOf2Capacity);
    RegisterTest("test_QueueInit_ZeroElementSize", test_QueueInit_ZeroElementSize);
    RegisterTest("test_QueueEnqueueDequeue_Basic", test_QueueEnqueueDequeue_Basic);
    RegisterTest("test_QueueEnqueue_NullQueue", test_QueueEnqueue_NullQueue);
    RegisterTest("test_QueueEnqueue_NullElement", test_QueueEnqueue_NullElement);
    RegisterTest("test_QueueDequeue_NullQueue", test_QueueDequeue_NullQueue);
    RegisterTest("test_QueueDequeue_NullElement", test_QueueDequeue_NullElement);
    RegisterTest("test_QueueDequeue_EmptyQueue", test_QueueDequeue_EmptyQueue);
    RegisterTest("test_QueueFillToCapacity", test_QueueFillToCapacity);
    RegisterTest("test_QueueEnqueue_FullQueue", test_QueueEnqueue_FullQueue);
    RegisterTest("test_QueueFIFOOrdering", test_QueueFIFOOrdering);
    RegisterTest("test_QueueCircularBehavior", test_QueueCircularBehavior);
    RegisterTest("test_QueuePeek_Basic", test_QueuePeek_Basic);
    RegisterTest("test_QueuePeek_EmptyQueue", test_QueuePeek_EmptyQueue);
    RegisterTest("test_QueuePeek_NullParameters", test_QueuePeek_NullParameters);
    RegisterTest("test_QueueClear", test_QueueClear);
    RegisterTest("test_QueueUint32", test_QueueUint32);
    RegisterTest("test_QueueStruct", test_QueueStruct);
    RegisterTest("test_QueueUtilities_NullQueue", test_QueueUtilities_NullQueue);
    RegisterTest("test_QueueMultipleCycles", test_QueueMultipleCycles);
    RegisterTest("test_QueuePeekMultiple", test_QueuePeekMultiple);
}
