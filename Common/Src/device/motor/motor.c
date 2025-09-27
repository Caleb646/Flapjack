#include "device/motor/motor.h"
#include "common.h"
#include "conf/board.h"
#include "conf/conf.h"
#include "conf/ids.h"
#include "hal.h"
#include "log/logger.h"
#include "mc/dshot.h"
#include "mem/mem.h"
#include "mem/vector.h"
#include "peripheral/dma.h"
#include "peripheral/timer.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>


#define MOTOR_VALID(pMOTOR) \
    ((pMOTOR) != NULL && (pMOTOR)->isInitialized == true)

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

eSTATUS_t MotorsApply (MotorApplyFn_t fn, void* pContext) {

    if (fn == NULL) {
        LOG_ERROR ("Received NULL function pointer");
        return eSTATUS_FAILURE;
    }

    for (uint32_t i = 0; i < MotorVector_Size (); ++i) {

        Motor_t* pMotor = MotorVector_At (i);
        if (MOTOR_VALID (pMotor) == true) {
            if (fn (pMotor, pContext) != eSTATUS_SUCCESS) {
                LOG_ERROR (
                "Failed to apply function to motor ID %u",
                pMotor->motorId
                );
                return eSTATUS_FAILURE;
            }
        }
    }
    return eSTATUS_SUCCESS;
}

eSTATUS_t MotorInit (MotorInitConf_t conf) {

    eSTATUS_t status                  = eSTATUS_SUCCESS;
    MotorBoardConf_t boardConf        = conf.boardConf;
    TimerBoardConf_t* pTimerBoardConf = boardConf.pTimerBoardConf;
    ServoBoardConf_t* pLinkedServoBoardConf = boardConf.pLinkedServoBoardConf;
    eDEVICE_ID_t id    = boardConf.motorId;
    uint8_t useDMA     = boardConf.useDMA;
    uint8_t dshotSpeed = boardConf.dshotSpeed;
    (void)dshotSpeed;
    float pidRollMix  = boardConf.pidRollMix;
    float pidPitchMix = boardConf.pidPitchMix;
    float pidYawMix   = boardConf.pidYawMix;

    if (pTimerBoardConf == NULL) {
        LOG_ERROR ("MotorInit: pTimerBoardConf is NULL");
        return eSTATUS_FAILURE;
    }

    Motor_t motor   = { 0 };
    Motor_t* pMotor = &motor;
    if (pMotor == NULL) {
        LOG_ERROR ("Failed to get Motor_t by ID");
        return eSTATUS_FAILURE;
    }

    memset (pMotor, 0, sizeof (Motor_t));
    pMotor->motorId           = id;
    pMotor->usingDMA          = useDMA;
    pMotor->pitchMix          = pidPitchMix;
    pMotor->yawMix            = pidYawMix;
    pMotor->rollMix           = pidRollMix;
    pMotor->curThrottle       = 0.0F;
    pMotor->curTargetThrottle = 0.0F;

    eTIMER_ID_t timerId = pTimerBoardConf->timerId;
    (void)timerId;
    eGPIO_ID_t gpioId = pTimerBoardConf->gpioId;
    (void)gpioId;

    if (useDMA == false) {
        DSHOT_INIT_BITBANG (&status, boardConf, *pTimerBoardConf);
        if (status != eSTATUS_SUCCESS) {
            LOG_ERROR ("Failed to create DShotInitConf_t for Motor_t");
            goto error;
        }
    } else {
        DSHOT_INIT_DMA (&status, boardConf, *pTimerBoardConf);
        if (status != eSTATUS_SUCCESS) {
            LOG_ERROR ("Failed to create DShotInitConf_t for Motor_t");
            goto error;
        }
    }

    if (pLinkedServoBoardConf != NULL) {
        pMotor->linkedServoId = pLinkedServoBoardConf->servoId;
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

    if (DShotStart (pMotor->motorId) != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to start DShot for Motor_t");
        return eSTATUS_FAILURE;
    }
    // Set initial throttle to minimum
    // return DShotWrite (&pMotor->dshot, DSHOT_MIN_THROTTLE);
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
    if (DShotStop (pMotor->motorId) != eSTATUS_SUCCESS) {
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

    float clippedThrottle =
    clipf32 (targetThrottle, MOTOR_MIN_THROTTLE, MOTOR_MAX_THROTTLE);
    pMotor->curThrottle       = clippedThrottle;
    pMotor->curTargetThrottle = targetThrottle;

    eSTATUS_t status =
    DShotWrite (pMotor->motorId, DSHOT_MIN_THROTTLE + (uint16_t)(clippedThrottle * (float)DSHOT_RANGE));
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

    eSTATUS_t status = DShotWrite (pMotor->motorId, command);
    if (status != eSTATUS_SUCCESS && status != eSTATUS_BUSY) {
        LOG_ERROR ("Failed to write command to motor");
        return status;
    }

    return eSTATUS_SUCCESS;
}