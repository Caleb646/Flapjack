#ifndef MOTION_CONTROL_ACTUATORS_H
#define MOTION_CONTROL_ACTUATORS_H

#include "common.h"
#include "conf/board.h"
#include "conf/conf.h"
#include "device/motor/motor.h"
#include "device/servo/servo.h"
#include "hal.h"
#include "mc/dshot.h"
#include "peripheral/dma.h"
#include "peripheral/timer.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

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

#ifdef UNIT_TEST
eSTATUS_t
ActuatorsMixPair (eDEVICE_ID_t servoId, eDEVICE_ID_t motorId, Vec3f pidAttitude, float tthrottle);
eSTATUS_t ActuatorsArm (void);
#endif // UNIT_TEST

eSTATUS_t ActuatorsInitMotor (MotorInitConf_t conf);
eSTATUS_t ActuatorsInitServo (ServoInitConf_t conf);
eSTATUS_t
ActuatorsInit (MotorBoardConf_t* pMotorBoardConfs, uint32_t numMotors, ServoBoardConf_t* pServoBoardConfs, uint32_t numServos);
eSTATUS_t ActuatorsStart (void);
eSTATUS_t ActuatorsStop (void);
eSTATUS_t ActuatorsWrite (Vec3f pidAttitude, float targetThrottle);
void ActuatorsLogData (void);

#define ACTUATORS_INIT_SERVO(pSTATUS, DEVICE_BOARD_CONF)  \
    do {                                                  \
        ServoInitConf_t conf = { 0 };                     \
        conf.boardConf       = (DEVICE_BOARD_CONF);       \
        *(pSTATUS)           = ActuatorsInitServo (conf); \
    } while (0)

#define ACTUATORS_INIT_MOTOR(pSTATUS, DEVICE_BOARD_CONF)  \
    do {                                                  \
        MotorInitConf_t conf = { 0 };                     \
        conf.boardConf       = (DEVICE_BOARD_CONF);       \
        *(pSTATUS)           = ActuatorsInitMotor (conf); \
    } while (0)


#define ACTUATORS_INIT(pSTATUS, pBOARD_CONF) \
    do {                                     \
        *(pSTATUS) = ActuatorsInit (         \
        (pBOARD_CONF)->pMotorBoardConfs,     \
        (pBOARD_CONF)->numMotors,            \
        (pBOARD_CONF)->pServoBoardConfs,     \
        (pBOARD_CONF)->numServos             \
        );                                   \
    } while (0)

#endif // MOTION_CONTROL_ACTUATORS_H
