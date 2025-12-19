
#include "device/servo/servo.h"
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


#define SERVO_VALID(pSERVO) ((pSERVO) != NULL && (pSERVO)->isInitialized == true)

VECTOR_DEFINE_STATIC_SHARED (Servo, Servo_t, SERVO_COUNT);

#ifndef UNIT_TEST
static float ServoAngle2PWM (Servo_t* pServo, float targetAngle);
#endif // UNIT_TEST

FJ_STATIC float ServoAngle2PWM (Servo_t* pServo, float targetAngle) {
    /*
     * NOTE: The servo on its own can move between -maxAngle and +maxAngle. But when placed
     * in the drone, it may only be able to move between -usableMaxAngle and +usableMaxAngle.
     *
     * Clip the target angle to the usable range first, then map it to the PWM duty cycle using the max angle.
     */

    // Handle asymmetric servo ranges: map negative angles to
    // [usLeftDutyCycle, usMiddleDutyCycle], positive to [usMiddleDutyCycle, usRightDutyCycle]
    if (targetAngle < 0) {
        return mapf32 (
        targetAngle,
        -pServo->maxAngle,
        0.0F,
        (float)pServo->usLeftDutyCycle,
        (float)pServo->usMiddleDutyCycle
        );
    }

    if (targetAngle > 0) {
        return mapf32 (
        targetAngle,
        0.0F,
        pServo->maxAngle,
        (float)pServo->usMiddleDutyCycle,
        (float)pServo->usRightDutyCycle
        );
    }

    return (float)pServo->usMiddleDutyCycle;
}

Servo_t* ServoGetById (eDEVICE_ID_t servoId) {

    for (uint32_t i = 0; i < ServoVector_Size (); ++i) {
        Servo_t* pServo = ServoVector_At (i);
        if (SERVO_VALID (pServo) && pServo->servoId == servoId) {
            return pServo;
        }
    }
    return NULL;
}

Vector_t* ServoGetAll (void) {
    return ServoVector_GetVector ();
}

// eSTATUS_t ServosApply (ServoApplyFn_t fn, void* pContext) {

//     if (fn == NULL) {
//         LOG_ERROR ("Received NULL function pointer");
//         return eSTATUS_FAIL;
//     }

//     for (uint32_t i = 0; i < ServoVector_Size (); ++i) {
//         Servo_t* pServo = ServoVector_At (i);
//         if (SERVO_VALID (pServo) == true) {
//             if (fn (pServo, pContext) != eSTATUS_OK) {
//                 LOG_ERROR (
//                 "Failed to apply function to servo ID %u",
//                 pServo->servoId
//                 );
//                 return eSTATUS_FAIL;
//             }
//         }
//     }
//     return eSTATUS_OK;
// }

eSTATUS_t ServoInit (ServoInitConf_t conf, Servo_t* pOutServo) {

    static bool isVectorInitialized = false;
    if (isVectorInitialized == false) {
        if (ServoVector_Init () != eSTATUS_OK) {
            LOG_ERROR ("Failed to initialize servo vector");
            return eSTATUS_FAIL;
        }
        isVectorInitialized = true;
    }

    eSTATUS_t status        = eSTATUS_OK;
    DeviceDesc_t deviceConf = conf.boardConf;
    eDEVICE_ID_t servoId    = deviceConf.deviceId;
    RETURN_IF (DEVICE_ID_IS_SERVO (servoId) == false, eSTATUS_FAIL, "Invalid servo ID: %u", servoId);

    ServoDeviceConf_t servoConf  = deviceConf.servo;
    TimerDesc_t* pTimerBoardConf = servoConf.pTimerBoardConf;
    RETURN_IF_NULL (pTimerBoardConf, eSTATUS_FAIL, "ServoInit: pTimerBoardConf is NULL");
    eTIMER_ID_t timerId                 = pTimerBoardConf->timerId;
    DeviceDesc_t* pLinkedMotorBoardConf = servoConf.pLinkedMotorBoardConf;

    uint32_t pwmFrequency = servoConf.pwmFreq;
    float pidRollMix      = servoConf.pidRollMix;
    float pidPitchMix     = servoConf.pidPitchMix;
    float pidYawMix       = servoConf.pidYawMix;

    Servo_t servo   = { 0 };
    Servo_t* pServo = &servo;
    if (pOutServo != NULL) {
        pServo = pOutServo;
    }
    memset (pServo, 0, sizeof (Servo_t));
    pServo->servoId           = servoId;
    pServo->usingDMA          = false;
    pServo->pitchMix          = pidPitchMix;
    pServo->yawMix            = pidYawMix;
    pServo->rollMix           = pidRollMix;
    pServo->curAngle          = 0.0F;
    pServo->curTargetAngle    = 0.0F;
    pServo->usLeftDutyCycle   = 500;
    pServo->usMiddleDutyCycle = 1500;
    pServo->usRightDutyCycle  = 2500;
    pServo->maxAngle          = 90.0F;
    pServo->usableMaxAngle    = 25.0F;

    status = TIMER_INIT_PWM (servoId, timerId, pwmFrequency, *pTimerBoardConf);
    GOTO_IF (FJ_FAIL (status), error, "Failed to initialize timer for Servo_t");
    pServo->pTimer = Timer_Get_ById (timerId);

    if (pLinkedMotorBoardConf != NULL) {
        pServo->linkedMotorId = pLinkedMotorBoardConf->deviceId;
    }

    /*
     * NOTE: When given a duty cycle of 2500us the servo spins CW (POV is
     * servo shaft is pointed to the ceiling and its being viewed from
     * above). For example, if the servo is mounted on the left side of the
     * drone (POV directly behind drone), pid will respond to a positive
     * roll with a negative pid roll. The left servo should respond to the
     * negative pid roll by spinning CW (positive angle, duty cycle >
     * 1600us) so the sign of servo pitch mix needs to be negative. This
     * flips the sign of the pid pitch from negative to positive so the
     * left servo spins CW.
     */
    pServo->isInitialized = true;
    if (ServoVector_PushBack (pServo) != eSTATUS_OK) {
        LOG_ERROR ("Failed to add Servo_t to vector");
        goto error;
    }
    return eSTATUS_OK;

error:
    memset (pServo, 0, sizeof (Servo_t));
    return eSTATUS_FAIL;
}

eSTATUS_t ServoStart (Servo_t* pServo) {

    if (SERVO_VALID (pServo) == false) {
        LOG_ERROR ("Failed to get Servo_t by ID");
        return eSTATUS_FAIL;
    }

    if (Timer_Start (pServo->pTimer, NULL, 0) != eSTATUS_OK) {
        LOG_ERROR ("Failed to start Timer for Servo_t");
        return eSTATUS_FAIL;
    }

    LOG_INFO ("Started Servo ID %u", pServo->servoId);
    // Set initial angle to middle position
    return ServoWrite (pServo, 0.0F);
}

eSTATUS_t ServoStop (Servo_t* pServo) {

    if (SERVO_VALID (pServo) == false) {
        LOG_ERROR ("Failed to get Servo_t by ID");
        return eSTATUS_FAIL;
    }

    if (Timer_Stop (pServo->pTimer) != eSTATUS_OK) {
        LOG_ERROR ("Failed to stop Timer for Servo_t");
        return eSTATUS_FAIL;
    }

    return eSTATUS_OK;
}

eSTATUS_t ServoWrite (Servo_t* pServo, float targetAngle) {

    if (SERVO_VALID (pServo) == false) {
        LOG_ERROR ("Failed to get Servo_t by ID");
        return eSTATUS_FAIL;
    }

    float usableMaxAngle   = pServo->usableMaxAngle;
    float clippedAngle     = clipf32 (targetAngle, -usableMaxAngle, usableMaxAngle);
    pServo->curAngle       = clippedAngle;
    pServo->curTargetAngle = targetAngle;
    return Timer_Write (pServo->pTimer, (uint32_t)ServoAngle2PWM (pServo, clippedAngle));
}