#ifndef MOTION_CONTROL_ACTUATORS_H
#define MOTION_CONTROL_ACTUATORS_H

#include "common.h"
#include "hal.h"
#include "motion_control/pwm.h"
#include <stdint.h>


typedef struct {
    uint32_t channel1;
    uint32_t channel2;
    uint32_t channel3;
    uint32_t channel4;
    uint32_t channel5;
    uint32_t channel6;
} RadioPWMChannels;

typedef struct {
    float rollP, rollI, rollD;
    float pitchP, pitchI, pitchD;
    float yawP, yawI, yawD;
    float integralLimit;
    Vec3f prevError;
    Vec3f prevIntegral;
} PIDContext;

typedef struct {
    uint32_t usLeftDutyCycle;
    uint32_t usMiddleDutyCycle;
    uint32_t usRightDutyCycle;
    float maxAngle;
    // Between -max angle and +max angle
    float curAngle;
    float pitchMix;
    float yawMix;
    float rollMix;
} ServoDescriptor;

typedef struct {
    PWMHandle pwmHandle;
    ServoDescriptor pwmDescriptor;
} Servo;

typedef struct {
    uint32_t usMinDutyCycle;
    uint32_t usMaxDutyCycle;
} MotorDescriptor;

typedef struct {
    PWMHandle pwmHandle;
    MotorDescriptor pwmDescriptor;
} Motor;

typedef struct {
    Vec3 forward;
    Vec3 up;
    Vec3 right;
} AxisMap;

STATUS_TYPE UpdateTargetAttitudeThrottle (
Vec3f maxAttitude,
RadioPWMChannels radio,
Vec3f* pOutputTargetAttitude,
float* pOutputThrottle);

STATUS_TYPE PIDUpdateAttitude (
PIDContext* pidContext,
Vec3f currentAttitude, // degrees
Vec3f targetAttitude,  // degrees
Vec3f maxAttitude,     // degrees
float dt,
Vec3f* pOutputPIDAttitude // degrees
);

STATUS_TYPE PIDInit (PIDContext* pContext);
#ifdef UNIT_TEST
float ServoAngle2PWM (ServoDescriptor* pServo, float targetAngle);
#endif // UNIT_TEST
STATUS_TYPE ServoMove2Angle (Servo* pServo, float targetAngle);
STATUS_TYPE TestServoMove2Angle (float targetAngle);

STATUS_TYPE PWMMixPIDnSend (Vec3f pidAttitude, float targetThrottle);
STATUS_TYPE ActuatorsInit (PWMHandle leftMotorPWM, PWMHandle leftServoPWM);

// void MotionControlUpdatePWM(
//     AxisMap axisConf, Vec3 mmVelSteps, Vec3 mmAngVelSteps, void *devs, uint32_t nDevs
// );

#endif // MOTION_CONTROL_ACTUATORS_H
