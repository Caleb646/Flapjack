#ifndef DEVICE_MOTOR_H
#define DEVICE_MOTOR_H

#include "conf/board.h"
#include "conf/conf.h"
#include "conf/ids.h"
#include "core/core.h"
#include "core/log/logger.h"
#include "core/stl/vector.h"
#include "hal.h"
#include "mc/dshot.h"
#include "peripheral/dma.h"
#include "peripheral/timer.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>


#define MOTOR_COUNT (MOTOR_MAX_MOTORS)

typedef uint8_t eMOTOR_CMD_t;
enum {
    eMOTOR_CMD_ARM    = 0x00,
    eMOTOR_CMD_DISARM = 0x00,
};

typedef struct {
    DevDesc_t* pDevDesc;
} MotorInitConf_t;

typedef struct Servo_s Servo_t;

typedef struct Motor_s {
    // eDEVICE_ID_t motorId;
    // eDEVICE_ID_t linkedServoId;
    // eTIMER_ID_t timerId;
    // float pitchMix;
    // float yawMix;
    // float rollMix;
    float curThrottle; // Between 0.0 and 1.0
    float curTargetThrottle;
    // bool isInitialized;
} Motor_t;

// typedef eSTATUS_t (*MotorApplyFn_t) (Motor_t* pMotor, void* pContext);

#ifdef UNIT_TEST

#endif // UNIT_TEST

Motor_t* MotorGetById (eDEVICE_ID_t motorId);
// Vector_t* MotorGetAll (void);
// eSTATUS_t MotorsApply (MotorApplyFn_t fn, void* pContext);
eSTATUS_t MotorInit (MotorInitConf_t conf, Motor_t* pOutMotor);
eSTATUS_t MotorStart (Motor_t* pMotor);
eSTATUS_t MotorStop (Motor_t* pMotor);
eSTATUS_t MotorWrite (Motor_t* pMotor, float targetThrottle);
eSTATUS_t MotorWriteCmd (Motor_t* pMotor, eMOTOR_CMD_t command);

#define MOTOR_START(MOTOR_ID)              MotorStart (MotorGetById (MOTOR_ID))
#define MOTOR_WRITE(MOTOR_ID, MOTOR_VALUE) MotorWrite (MotorGetById (MOTOR_ID), MOTOR_VALUE)
#define MOTOR_WRITE_CMD(MOTOR_ID, COMMAND) MotorWriteCmd (MotorGetById (MOTOR_ID), COMMAND)

#endif // DEVICE_MOTOR_H