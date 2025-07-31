#ifndef MOTION_CONTROL_ACTUATORS_H
#define MOTION_CONTROL_ACTUATORS_H

#include "common.h"
#include "dma.h"
#include "hal.h"
#include "mc/dshot.h"
#include "mc/pwm.h"
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

// eSTATUS_t UpdateTargetAttitudeThrottle (
// Vec3f maxAttitude,
// RadioPWMChannels radio,
// Vec3f* pOutputTargetAttitude,
// float* pOutputThrottle);
// eSTATUS_t PIDInit (PIDContext* pContext);

eSTATUS_t PIDUpdateAttitude (
PIDContext* pidContext,
Vec3f currentAttitude, // degrees
Vec3f targetAttitude,  // degrees
Vec3f maxAttitude,     // degrees
float dt,
Vec3f* pOutputPIDAttitude // degrees
);

eSTATUS_t ServoInit (PWMConfig config, Servo* pOutServo);
eSTATUS_t ServoStart (Servo* pServo);
eSTATUS_t ServoWrite (Servo* pServo, float targetAngle);

eSTATUS_t MotorInit (MotorConfig config, Motor* pOutMotor);
eSTATUS_t MotorStart (Motor* pMotor);
eSTATUS_t MotorWrite (Motor* pMotor, float motorValue);

eSTATUS_t ActuatorsInit (PWMConfig left_ServoPWM, MotorConfig left_Motor);
eSTATUS_t ActuatorsStart (void);
eSTATUS_t ActuatorsStop (void);
eSTATUS_t ActuatorsWrite (Vec3f pidAttitude, float targetThrottle);
Servo* ActuatorsGetLeftServo (void);


#ifdef UNIT_TEST
float ServoAngle2PWM (Servo* pServo, float targetAngle);
eSTATUS_t ActuatorsMixPair (Servo* pServo, Motor* pMotor, Vec3f pidAttitude, float tthrottle);
eSTATUS_t ActuatorsArm (void);
#endif // UNIT_TEST


#endif // MOTION_CONTROL_ACTUATORS_H
