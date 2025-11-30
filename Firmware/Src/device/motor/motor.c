#include "device/motor/motor.h"
#include "common.h"
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


#define MOTOR_VALID(pMOTOR) ((pMOTOR) != NULL && (pMOTOR)->isInitialized)

VECTOR_DEFINE_STATIC_SHARED (Motor, Motor_t, MOTOR_COUNT);

Motor_t* MotorGetById (eDEVICE_ID_t motorId) {

    for (uint32_t i = 0; i < MotorVector_Size (); ++i) {
        Motor_t* pMotor = MotorVector_At (i);
        if (MOTOR_VALID (pMotor) && pMotor->motorId == motorId) {
            return pMotor;
        }
    }
    return NULL;
}

Vector_t* MotorGetAll (void) {
    return MotorVector_GetVector ();
}

eSTATUS_t MotorInit (MotorInitConf_t conf, Motor_t* pOutMotor) {

    static bool isVectorInitialized = false;
    if (isVectorInitialized == false) {
        if (MotorVector_Init () != eSTATUS_SUCCESS) {
            LOG_ERROR ("Failed to initialize motor vector");
            return eSTATUS_FAILURE;
        }
        isVectorInitialized = true;
    }

    eSTATUS_t status     = eSTATUS_SUCCESS;
    DeviceDesc_t device  = conf.boardConf;
    eDEVICE_ID_t motorId = device.deviceId;
    RETURN_IF (DEVICE_ID_IS_MOTOR (motorId) == false, eSTATUS_FAILURE, "Invalid motor ID: %u", motorId);

    MotorDesc_t motorConf               = device.motor;
    bool usingDMA                       = motorConf.useDMA;
    TimerDesc_t* pTimerBoardConf        = motorConf.pTimerBoardConf;
    DeviceDesc_t* pLinkedServoBoardConf = motorConf.pLinkedServoBoardConf;
    // uint8_t dshotSpeed                       = motor.dshotSpeed;
    // (void)dshotSpeed;
    float pidRollMix  = motorConf.pidRollMix;
    float pidPitchMix = motorConf.pidPitchMix;
    float pidYawMix   = motorConf.pidYawMix;

    RETURN_IF_NULL (pTimerBoardConf, eSTATUS_FAILURE, "MotorInit: pTimerBoardConf is NULL");

    Motor_t motor   = { 0 };
    Motor_t* pMotor = &motor;
    if (pOutMotor != NULL) {
        pMotor = pOutMotor;
    }

    memset (pMotor, 0, sizeof (Motor_t));
    pMotor->motorId           = motorId;
    pMotor->usingDMA          = usingDMA;
    pMotor->pitchMix          = pidPitchMix;
    pMotor->yawMix            = pidYawMix;
    pMotor->rollMix           = pidRollMix;
    pMotor->curThrottle       = 0.0F;
    pMotor->curTargetThrottle = 0.0F;

    status = DSHOT_INIT (device);
    RETURN_IF (FJ_FAIL (status), eSTATUS_FAILURE, "Failed to initialize DShot for motor ID %u", motorId);

    if (pLinkedServoBoardConf != NULL) {
        pMotor->linkedServoId = pLinkedServoBoardConf->deviceId;
    }

    pMotor->isInitialized = true;
    if (MotorVector_PushBack (pMotor) != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to add Motor_t to vector");
        goto error;
    }
    return eSTATUS_SUCCESS;

error:
    memset (pMotor, 0, sizeof (Motor_t));
    return eSTATUS_FAILURE;
}

eSTATUS_t MotorStart (Motor_t* pMotor) {

    if (MOTOR_VALID (pMotor) == false) {
        LOG_ERROR ("Received invalid motor pointer");
        return eSTATUS_FAILURE;
    }

    if (DSHOT_START (pMotor->motorId) != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to start DShot for Motor_t");
        return eSTATUS_FAILURE;
    }
    // Set initial throttle to minimum
    // return DShotWrite (&pMotor->dshot, DSHOT_MIN_THROTTLE);
    LOG_INFO ("Started Motor ID %u", pMotor->motorId);
    return eSTATUS_SUCCESS;
}

eSTATUS_t MotorStop (Motor_t* pMotor) {

    if (MOTOR_VALID (pMotor) == false) {
        LOG_ERROR ("Received invalid motor pointer");
        return eSTATUS_FAILURE;
    }

    /*
     * NOTE: The motor's ESC will automatically stop the motor
     * when the PWM signal has stopped for more than 5-10ms.
     */
    if (DSHOT_STOP (pMotor->motorId) != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to stop dshot");
        return eSTATUS_FAILURE;
    }

    return eSTATUS_SUCCESS;
}

/*
 * throttle is between 0.0F and 1.0F
 */
eSTATUS_t MotorWrite (Motor_t* pMotor, float targetThrottle) {

    if (MOTOR_VALID (pMotor) == false) {
        LOG_ERROR ("Received invalid motor pointer");
        return eSTATUS_FAILURE;
    }

    if (targetThrottle < 0.0F || targetThrottle > 1.0F) {
        LOG_ERROR ("Motor_t throttle out of range: %u", (uint16_t)(targetThrottle * 100.0F));
        return eSTATUS_FAILURE;
    }

    float clippedThrottle     = clipf32 (targetThrottle, MOTOR_MIN_THROTTLE, MOTOR_MAX_THROTTLE);
    pMotor->curThrottle       = clippedThrottle;
    pMotor->curTargetThrottle = targetThrottle;

    eSTATUS_t status =
    DSHOT_WRITE (pMotor->motorId, DSHOT_MIN_THROTTLE + (uint16_t)(clippedThrottle * (float)DSHOT_RANGE));
    if (status != eSTATUS_SUCCESS && status != eSTATUS_BUSY) {
        LOG_ERROR ("Failed to write to motor");
        return status;
    }
    /*
     * NOTE: DShotWrite returns eSTATUS_BUSY when a write is in progress.
     * Don't consider this a failure.
     */
    return eSTATUS_SUCCESS;
}

eSTATUS_t MotorWriteCmd (Motor_t* pMotor, eMOTOR_CMD_t command) {

    if (MOTOR_VALID (pMotor) == false) {
        LOG_ERROR ("Received invalid motor pointer");
        return eSTATUS_FAILURE;
    }

    switch (command) {
    // NOTE: The arm and disarm command values are the same and depend on the ESC's current state.
    case eMOTOR_CMD_ARM: break;
    // case eMOTOR_CMD_DISARM: break;
    default: LOG_ERROR ("Unknown motor command"); return eSTATUS_FAILURE;
    }

    eSTATUS_t status = DSHOT_WRITE (pMotor->motorId, command);
    if (status != eSTATUS_SUCCESS && status != eSTATUS_BUSY) {
        LOG_ERROR ("Failed to write command to motor");
        return status;
    }

    return eSTATUS_SUCCESS;
}