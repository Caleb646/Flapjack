#ifndef MOTION_CONTROL_ACTUATORS_H
#define MOTION_CONTROL_ACTUATORS_H

#include "common.h"
#include "dma.h"
#include "hal.h"
#include "motion_control/dshot.h"
#include "motion_control/pwm.h"
#include <stdint.h>

#define MOTOR_CREATE_CONF(PTR_TIMER, CHANNEL_ID, DMA_STREAM, DMA_REQUEST_ID) \
    { .pwm = PWM_CREATE_CONF (PTR_TIMER, CHANNEL_ID, 0, FALSE),              \
      .dma = DMA_CREATE_PWM_CONF (DMA_STREAM, eDMA_DIRECTION_MEMORY_TO_PERIPH, eDMA_PRIORITY_HIGH, DMA_REQUEST_ID) }

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
    PWMHandle pwm;
    ServoDescriptor desc;
} Servo;

typedef struct {
    PWMConfig pwm;
    DMAConfig dma;
} MotorConfig;

typedef struct {
    uint32_t unused;
    // uint32_t usMaxDutyCycle;
} MotorDescriptor;

typedef struct {
    DShotHandle dshot;
    MotorDescriptor desc;
} Motor;

typedef struct {
    Vec3 forward;
    Vec3 up;
    Vec3 right;
} AxisMap;

// STATUS_TYPE UpdateTargetAttitudeThrottle (
// Vec3f maxAttitude,
// RadioPWMChannels radio,
// Vec3f* pOutputTargetAttitude,
// float* pOutputThrottle);
// STATUS_TYPE PIDInit (PIDContext* pContext);

STATUS_TYPE PIDUpdateAttitude (
PIDContext* pidContext,
Vec3f currentAttitude, // degrees
Vec3f targetAttitude,  // degrees
Vec3f maxAttitude,     // degrees
float dt,
Vec3f* pOutputPIDAttitude // degrees
);

STATUS_TYPE ServoInit (PWMConfig config, Servo* pOutServo);
STATUS_TYPE ServoStart (Servo* pServo);
STATUS_TYPE ServoWrite (Servo* pServo, float targetAngle);

STATUS_TYPE MotorInit (MotorConfig config, Motor* pOutMotor);
STATUS_TYPE MotorStart (Motor* pMotor);
STATUS_TYPE MotorWrite (Motor* pMotor, float motorValue);

STATUS_TYPE ActuatorsInit (PWMConfig left_ServoPWM, MotorConfig left_Motor);
STATUS_TYPE ActuatorsStart (void);
STATUS_TYPE ActuatorsWrite (Vec3f pidAttitude, float targetThrottle);
STATUS_TYPE ActuatorsArm (void);
Servo* ActuatorsGetLeftServo (void);


#ifdef UNIT_TEST
float ServoAngle2PWM (Servo* pServo, float targetAngle);
STATUS_TYPE
ActuatorsMixPair (Servo* pServo, Motor* pMotor, Vec3f pidAttitude, float targetThrottle);
#endif // UNIT_TEST


#endif // MOTION_CONTROL_ACTUATORS_H
