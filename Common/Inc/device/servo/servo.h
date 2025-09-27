
#ifndef DEVICE_SERVO_H
#define DEVICE_SERVO_H

#include "common.h"
#include "conf/board.h"
#include "conf/conf.h"
#include "conf/ids.h"
#include "hal.h"
#include "log/logger.h"
#include "mc/dshot.h"
#include "mem/vector.h"
#include "peripheral/dma.h"
#include "peripheral/timer.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define SERVO_COUNT (eSERVO_ID_MAX)

typedef struct {
    ServoBoardConf_t boardConf;
} ServoInitConf_t;

typedef struct Motor_s Motor_t;

typedef struct Servo_s {
    eDEVICE_ID_t servoId;
    eDEVICE_ID_t linkedMotorId;
    eTIMER_ID_t timerId;
    uint32_t usLeftDutyCycle;
    uint32_t usMiddleDutyCycle;
    uint32_t usRightDutyCycle;
    float maxAngle;
    float usableMaxAngle;
    float pitchMix;
    float yawMix;
    float rollMix;
    // Between -usableMaxAngle angle and +usableMaxAngle angle
    float curAngle;
    float curTargetAngle;
    bool usingDMA;
    bool isInitialized;
} Servo_t;

typedef eSTATUS_t (*ServoApplyFn_t) (Servo_t* pServo, void* pContext);

#ifdef UNIT_TEST
float ServoAngle2PWM (Servo_t* pServo, float targetAngle);
#endif // UNIT_TEST

Servo_t* ServoGetById (eDEVICE_ID_t servoId);
Vector_t* ServoGetAll (void);
eSTATUS_t ServosApply (ServoApplyFn_t fn, void* pContext);
eSTATUS_t ServoInit (ServoInitConf_t conf);
eSTATUS_t ServoStart (Servo_t* pServo);
eSTATUS_t ServoStop (Servo_t* pServo);
eSTATUS_t ServoWrite (Servo_t* pServo, float targetAngle);

#define SERVO_INIT(pSTATUS, DEVICE_BOARD_CONF)      \
    do {                                            \
        ServoInitConf_t conf = { 0 };               \
        conf.boardConf       = (DEVICE_BOARD_CONF); \
        *(pSTATUS)           = ServoInit (conf);    \
    } while (0)

#define SERVO_START(SERVO_ID) ServoStart (ServoGetById (SERVO_ID))
#define SERVO_WRITE(SERVO_ID, TARGET_ANGLE) \
    ServoWrite (ServoGetById (SERVO_ID), TARGET_ANGLE)

#endif // DEVICE_SERVO_H