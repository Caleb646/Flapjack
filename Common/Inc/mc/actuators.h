#ifndef MOTION_CONTROL_ACTUATORS_H
#define MOTION_CONTROL_ACTUATORS_H

#include "common.h"
#include "conf.h"
#include "dma.h"
#include "hal.h"
#include "mc/dshot.h"
#include "mc/pwm.h"
#include <stdint.h>


#define MOTOR_CREATE_CONF(PTR_TIMER, CHANNEL_ID, DMA_STREAM, DMA_REQUEST_ID) \
    { .pwm = PWM_CREATE_CONF (PTR_TIMER, CHANNEL_ID, 0, FALSE),              \
      .dma = DMA_CREATE_PWM_CONF (DMA_STREAM, eDMA_DIRECTION_MEMORY_TO_PERIPH, eDMA_PRIORITY_HIGH, DMA_REQUEST_ID) }

#define PID_INIT(PID_CONTEXT)                                 \
    PID_CONTEXT.rollP          = PID_STARTING_ROLL_P;         \
    PID_CONTEXT.rollI          = PID_STARTING_ROLL_I;         \
    PID_CONTEXT.rollD          = PID_STARTING_ROLL_D;         \
    PID_CONTEXT.pitchP         = PID_STARTING_PITCH_P;        \
    PID_CONTEXT.pitchI         = PID_STARTING_PITCH_I;        \
    PID_CONTEXT.pitchD         = PID_STARTING_PITCH_D;        \
    PID_CONTEXT.yawP           = PID_STARTING_YAW_P;          \
    PID_CONTEXT.yawI           = PID_STARTING_YAW_I;          \
    PID_CONTEXT.yawD           = PID_STARTING_YAW_D;          \
    PID_CONTEXT.integralLimit  = PID_STARTING_INTEGRAL_LIMIT; \
    PID_CONTEXT.prevError.x    = 0.0F;                        \
    PID_CONTEXT.prevError.y    = 0.0F;                        \
    PID_CONTEXT.prevError.z    = 0.0F;                        \
    PID_CONTEXT.prevIntegral.x = 0.0F;                        \
    PID_CONTEXT.prevIntegral.y = 0.0F;                        \
    PID_CONTEXT.prevIntegral.z = 0.0F;

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
    eSERVO_ID_t id;
    uint32_t usLeftDutyCycle;
    uint32_t usMiddleDutyCycle;
    uint32_t usRightDutyCycle;
    float maxAngle;
    float usableMaxAngle;
    float pitchMix;
    float yawMix;
    float rollMix;
} ServoDescriptor;

typedef struct {
    PWMHandle pwm;
    ServoDescriptor desc;
    // Between -usableMaxAngle angle and +usableMaxAngle angle
    float curAngle;
    float curTargetAngle;
} Servo;

typedef uint8_t eMOTOR_CMD_t;
enum {
    eMOTOR_CMD_ARM    = 0x00,
    eMOTOR_CMD_DISARM = 0x00,
};

typedef struct {
    PWMConfig pwm;
    DMAConfig dma;
} MotorConfig;

typedef struct {
    eSERVO_ID_t id;
    float pitchMix;
    float yawMix;
    float rollMix;
} MotorDescriptor;

typedef struct {
    DShotHandle dshot;
    MotorDescriptor desc;
    float curThrottle; // Between 0.0 and 1.0
    float curTargetThrottle;
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

#ifdef UNIT_TEST

float ServoAngle2PWM (Servo* pServo, float targetAngle);

#endif // UNIT_TEST

eSTATUS_t ServoInit (eACTUATOR_ID_t id, PWMConfig config, Servo* pOutServo);
eSTATUS_t ServoStart (Servo* pServo);
eSTATUS_t ServoWrite (Servo* pServo, float targetAngle);

#ifdef UNIT_TEST

#endif // UNIT_TEST

eSTATUS_t MotorInit (eACTUATOR_ID_t id, MotorConfig config, Motor* pOutMotor);
eSTATUS_t MotorStart (Motor* pMotor);
eSTATUS_t MotorWrite (Motor* pMotor, float motorValue);
eSTATUS_t MotorWriteCmd (Motor* pMotor, eMOTOR_CMD_t command);

#ifdef UNIT_TEST

eSTATUS_t ActuatorsMixPair (Servo* pServo, Motor* pMotor, Vec3f pidAttitude, float tthrottle);
eSTATUS_t ActuatorsArm (void);

#endif // UNIT_TEST

eSTATUS_t ActuatorsInit (PWMConfig left_ServoPWM, MotorConfig left_Motor);
eSTATUS_t ActuatorsStart (void);
eSTATUS_t ActuatorsStop (void);
eSTATUS_t ActuatorsWrite (Vec3f pidAttitude, float targetThrottle);
Servo* ActuatorsGetLeftServo (void);
void ActuatorsLogData (void);

#endif // MOTION_CONTROL_ACTUATORS_H
