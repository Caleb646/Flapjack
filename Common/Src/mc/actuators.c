#include "mc/actuators.h"
#include "common.h"
#include "conf/conf.h"
#include "log/logger.h"
#include "mc/dshot.h"
#include "peripheral/gpio.h"
#include "peripheral/timer.h"
#include <string.h>

eSTATUS_t PIDUpdateAttitude (
PIDContext* pidContext,
Vec3f currentAttitude,    // degrees
Vec3f targetAttitude,     // degrees
Vec3f maxAttitude,        // degrees
float dt,                 // seconds
Vec3f* pOutputPIDAttitude // degrees
) {

    if (pidContext == NULL || pOutputPIDAttitude == NULL) {
        LOG_ERROR ("PIDContext or output pointer is NULL");
        return eSTATUS_FAILURE;
    }

    if (dt == 0.0F) {
        LOG_ERROR ("dt cannot be zero");
        return eSTATUS_FAILURE;
    }

    float P            = pidContext->rollP;
    float I            = pidContext->rollI;
    float D            = pidContext->rollD;
    float rollError    = targetAttitude.roll - currentAttitude.roll;
    float rollIntegral = clipf32 (
    pidContext->prevIntegral.roll + rollError * dt,
    -pidContext->integralLimit,
    pidContext->integralLimit
    );
    float rollDerivative = (rollError - pidContext->prevError.roll) / dt;
    // pOutputPIDAttitude->roll = 0.01f * (P * rollError + I * rollIntegral - D * rollDerivative);

    // Scale PID output between -1 and 1
    pOutputPIDAttitude->roll = clipf32 (
                               (P * rollError + I * rollIntegral - D * rollDerivative),
                               -maxAttitude.roll,
                               maxAttitude.roll
                               ) /
                               maxAttitude.roll;

    P                   = pidContext->pitchP;
    I                   = pidContext->pitchI;
    D                   = pidContext->pitchD;
    float pitchError    = targetAttitude.pitch - currentAttitude.pitch;
    float pitchIntegral = clipf32 (
    pidContext->prevIntegral.pitch + pitchError * dt,
    -pidContext->integralLimit,
    pidContext->integralLimit
    );
    float pitchDerivative = (pitchError - pidContext->prevError.pitch) / dt;
    // pOutputPIDAttitude->pitch = 0.01f * (P * pitchError + I * pitchIntegral - D * pitchDerivative);

    // Scale PID output between -1 and 1
    pOutputPIDAttitude->pitch =
    clipf32 (
    (P * pitchError + I * pitchIntegral - D * pitchDerivative),
    -maxAttitude.pitch,
    maxAttitude.pitch
    ) /
    maxAttitude.pitch;

    P                 = pidContext->yawP;
    I                 = pidContext->yawI;
    D                 = pidContext->yawD;
    float yawError    = targetAttitude.yaw - currentAttitude.yaw;
    float yawIntegral = clipf32 (
    pidContext->prevIntegral.yaw + yawError * dt,
    -pidContext->integralLimit,
    pidContext->integralLimit
    );
    float yawDerivative = (yawError - pidContext->prevError.yaw) / dt;
    // pOutputPIDAttitude->yaw = 0.01f * (P * yawError + I * yawIntegral - D * yawDerivative);

    // Scale PID output between -1 and 1
    pOutputPIDAttitude->yaw = clipf32 (
                              (P * yawError + I * yawIntegral - D * yawDerivative),
                              -maxAttitude.yaw,
                              maxAttitude.yaw
                              ) /
                              maxAttitude.yaw;

    pidContext->prevIntegral.roll  = rollIntegral;
    pidContext->prevIntegral.pitch = pitchIntegral;
    pidContext->prevIntegral.yaw   = yawIntegral;

    pidContext->prevError.roll  = rollError;
    pidContext->prevError.pitch = pitchError;
    pidContext->prevError.yaw   = yawError;

    return eSTATUS_SUCCESS;
}

#ifndef UNIT_TEST

static float ServoAngle2PWM (Servo* pServo, float targetAngle);

#endif // UNIT_TEST

static Servo gServos[eSERVO_ID_MAX] = { 0 };
static Motor gMotors[eMOTOR_ID_MAX] = { 0 };

STATIC_TESTABLE_DECL float ServoAngle2PWM (Servo* pServo, float targetAngle) {

    if (pServo == NULL) {
        LOG_ERROR ("ServoDescriptor is not valid");
        return 0.0F;
    }
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

static Servo* ServoGetById (eDEVICE_ID_t deviceId) {

    if (DEVICE_ID_IS_SERVO (deviceId) == FALSE) {
        LOG_ERROR ("deviceId is not servo");
        return NULL;
    }

    return &gServos[SERVO_ID2IDX (deviceId)];
}

eSTATUS_t ServoInit (ServoInitConf_t conf) {

    eSTATUS_t status                  = eSTATUS_SUCCESS;
    ServoBoardConf_t boardConf        = conf.boardConf;
    TimerBoardConf_t* pTimerBoardConf = boardConf.pTimerBoardConf;
    MotorBoardConf_t* pLinkedMotorBoardConf = boardConf.pLinkedMotorBoardConf;
    (void)pLinkedMotorBoardConf;
    eDEVICE_ID_t id       = boardConf.servoId;
    uint32_t pwmFrequency = boardConf.pwmFrequency;
    float pidRollMix      = boardConf.pidRollMix;
    float pidPitchMix     = boardConf.pidPitchMix;
    float pidYawMix       = boardConf.pidYawMix;

    if (pTimerBoardConf == NULL) {
        LOG_ERROR ("ServoInit: pTimerBoardConf is NULL");
        return eSTATUS_FAILURE;
    }

    eTIMER_ID_t timerId = pTimerBoardConf->timerId;
    eGPIO_ID_t gpioId   = pTimerBoardConf->gpioId;
    (void)gpioId;

    Servo* pServo = ServoGetById (id);
    if (pServo == NULL) {
        LOG_ERROR ("Failed to get Servo by ID");
        return eSTATUS_FAILURE;
    }
    memset (pServo, 0, sizeof (Servo));
    pServo->id                = id;
    pServo->timerId           = timerId;
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

    TIMER_INIT_PWM (&status, id, timerId, pwmFrequency, *pTimerBoardConf);
    if (status != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to initialize timer for Servo");
        goto error;
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
    return eSTATUS_SUCCESS;

error:
    memset (pServo, 0, sizeof (Servo));
    return eSTATUS_FAILURE;
}

eSTATUS_t ServoStart (eDEVICE_ID_t servoId) {

    Servo* pServo = ServoGetById (servoId);
    if (pServo == NULL) {
        LOG_ERROR ("Failed to get Servo by ID");
        return eSTATUS_FAILURE;
    }

    eTIMER_ID_t timerId = pServo->timerId;
    if (TimerStart (timerId, NULL, 0) != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to start Timer for Servo");
        return eSTATUS_FAILURE;
    }

    // Set initial angle to middle position
    return ServoWrite (servoId, 0.0F);
}

eSTATUS_t ServoStop (eDEVICE_ID_t servoId) {

    Servo* pServo = ServoGetById (servoId);
    if (pServo == NULL) {
        LOG_ERROR ("Failed to get Servo by ID");
        return eSTATUS_FAILURE;
    }

    eTIMER_ID_t timerId = pServo->timerId;
    if (TimerStop (timerId) != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to stop Timer for Servo");
        return eSTATUS_FAILURE;
    }

    return eSTATUS_SUCCESS;
}

eSTATUS_t ServoWrite (eDEVICE_ID_t servoId, float targetAngle) {

    Servo* pServo = ServoGetById (servoId);
    if (pServo == NULL) {
        LOG_ERROR ("Failed to get Servo by ID");
        return eSTATUS_FAILURE;
    }

    eTIMER_ID_t timerId  = pServo->timerId;
    float usableMaxAngle = pServo->usableMaxAngle;
    float clippedAngle = clipf32 (targetAngle, -usableMaxAngle, usableMaxAngle);
    pServo->curAngle       = clippedAngle;
    pServo->curTargetAngle = targetAngle;

    return TimerWrite (timerId, (uint32_t)ServoAngle2PWM (pServo, clippedAngle));
}


#ifndef UNIT_TEST

#endif // UNIT_TEST

static Motor* MotorGetById (eDEVICE_ID_t deviceId) {

    if (DEVICE_ID_IS_MOTOR (deviceId) == FALSE) {
        return NULL;
    }

    return &gMotors[MOTOR_ID2IDX (deviceId)];
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

    Motor* pMotor = MotorGetById (id);
    if (pMotor == NULL) {
        LOG_ERROR ("Failed to get Motor by ID");
        return eSTATUS_FAILURE;
    }

    memset (pMotor, 0, sizeof (Motor));
    pMotor->id                = id;
    pMotor->pitchMix          = pidPitchMix;
    pMotor->yawMix            = pidYawMix;
    pMotor->rollMix           = pidRollMix;
    pMotor->curThrottle       = 0.0F;
    pMotor->curTargetThrottle = 0.0F;

    eTIMER_ID_t timerId = pTimerBoardConf->timerId;
    (void)timerId;
    eGPIO_ID_t gpioId = pTimerBoardConf->gpioId;

    if (useDMA == FALSE) {
        DSHOT_INIT_BITBANG (&status, boardConf, *pTimerBoardConf);
        if (status != eSTATUS_SUCCESS) {
            LOG_ERROR ("Failed to create DShotInitConf_t for Motor");
            goto error;
        }
    } else {
        DSHOT_INIT_DMA (&status, boardConf, *pTimerBoardConf);
        if (status != eSTATUS_SUCCESS) {
            LOG_ERROR ("Failed to create DShotInitConf_t for Motor");
            goto error;
        }
    }

    if (pLinkedServoBoardConf != NULL) {
        pMotor->linkedServoId = pLinkedServoBoardConf->servoId;
    }

    return eSTATUS_SUCCESS;

error:
    memset (pMotor, 0, sizeof (Motor));
    return eSTATUS_FAILURE;
}

eSTATUS_t MotorStart (eDEVICE_ID_t motorId) {

    if (DEVICE_ID_IS_MOTOR (motorId) == FALSE) {
        LOG_ERROR ("deviceId is not motor");
        return eSTATUS_FAILURE;
    }

    if (DShotStart (motorId) != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to start DShot for Motor");
        return eSTATUS_FAILURE;
    }
    // Set initial throttle to minimum
    // return DShotWrite (&pMotor->dshot, DSHOT_MIN_THROTTLE);
    return eSTATUS_SUCCESS;
}

eSTATUS_t MotorStop (eDEVICE_ID_t motorId) {

    /*
     * NOTE: The motor's ESC will automatically stop the motor
     * when the PWM signal has stopped for more than 5-10ms.
     */
    if (DShotStop (motorId) != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to stop dshot");
        return eSTATUS_FAILURE;
    }

    return eSTATUS_SUCCESS;
}

/*
 * throttle is between 0.0F and 1.0F
 */
eSTATUS_t MotorWrite (eDEVICE_ID_t motorId, float targetThrottle) {

    Motor* pMotor = MotorGetById (motorId);
    if (pMotor == NULL) {
        LOG_ERROR ("Received NULL pointer for Motor");
        return eSTATUS_FAILURE;
    }

    if (targetThrottle < 0.0F || targetThrottle > 1.0F) {
        LOG_ERROR ("Motor throttle out of range: %u", (uint16_t)(targetThrottle * 100.0F));
        return eSTATUS_FAILURE;
    }

    float clippedThrottle =
    clipf32 (targetThrottle, MOTOR_MIN_THROTTLE, MOTOR_MAX_THROTTLE);
    pMotor->curThrottle       = clippedThrottle;
    pMotor->curTargetThrottle = targetThrottle;

    eSTATUS_t status =
    DShotWrite (motorId, DSHOT_MIN_THROTTLE + (uint16_t)(clippedThrottle * (float)DSHOT_RANGE));
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

eSTATUS_t MotorWriteCmd (eDEVICE_ID_t motorId, eMOTOR_CMD_t command) {


    if (DEVICE_ID_IS_MOTOR (motorId) == FALSE) {
        LOG_ERROR ("deviceId is not motor");
        return eSTATUS_FAILURE;
    }

    switch (command) {
    // NOTE: The arm and disarm command values are the same and depend on the ESC's current state.
    case eMOTOR_CMD_ARM: break;
    // case eMOTOR_CMD_DISARM: break;
    default: LOG_ERROR ("Unknown motor command"); return eSTATUS_FAILURE;
    }

    eSTATUS_t status = DShotWrite (motorId, command);
    if (status != eSTATUS_SUCCESS && status != eSTATUS_BUSY) {
        LOG_ERROR ("Failed to write command to motor");
        return status;
    }

    return eSTATUS_SUCCESS;
}

// NOLINTBEGIN(cppcoreguidelines-avoid-non-const-global-variables)
static Motor gLeftMotor;
static Servo gLeftServo;
// NOLINTEND(cppcoreguidelines-avoid-non-const-global-variables)

#ifndef UNIT_TEST

static eSTATUS_t
ActuatorsMixPair (eDEVICE_ID_t servoId, eDEVICE_ID_t motorId, Vec3f pidAttitude, float targetThrottle);
static eSTATUS_t ActuatorsArm (void);

#endif // UNIT_TEST

/*
 * \param pidAttitude roll, pitch, and yaw are between -1 and 1
 */
STATIC_TESTABLE_DECL eSTATUS_t
ActuatorsMixPair (eDEVICE_ID_t servoId, eDEVICE_ID_t motorId, Vec3f pidAttitude, float targetThrottle) {
    /*
     *  Motor Mixing
     */
    Motor* pMotor       = MotorGetById (motorId);
    float mPitchMix     = pMotor->pitchMix;
    float mYawMix       = pMotor->yawMix;
    float mRollMix      = pMotor->rollMix;
    float mixedThrottle = targetThrottle; // between 0 and 1
    /*
     * NOTE: A PID pitch value should always increase the throttle of both
     * motors regardless of the sign of the PID pitch value.
     */
    mixedThrottle += mPitchMix * ABS_F32 (pidAttitude.pitch) +
                     mRollMix * pidAttitude.roll + mYawMix * pidAttitude.yaw;
    mixedThrottle = clipf32 (mixedThrottle, 0.0F, 1.0F);

    /*
     *  Servo Mixing
     */
    Servo* pServo    = ServoGetById (servoId);
    float sPitchMix  = pServo->pitchMix;
    float sYawMix    = pServo->yawMix;
    float sRollMix   = pServo->rollMix;
    float mixedAngle = sPitchMix * pidAttitude.pitch +
                       sRollMix * pidAttitude.roll + sYawMix * pidAttitude.yaw;
    // NOTE: Maybe Roll should have a negative impact on target angle.
    // Meaning the magnitude of the target angle is closer to 0 the larger
    // pid roll is.
    mixedAngle = clipf32 (mixedAngle, -1.0F, 1.0F) * pServo->maxAngle;

    /* Motor throttle should be between 0 and 1 */
    MotorWrite (motorId, mixedThrottle);
    ServoWrite (servoId, mixedAngle);
    return eSTATUS_SUCCESS;
}

/*
 * \brief After motors are armed, a motor write has to be issued at
 * least every 5ms or the motor ESC will stop the motor.
 */
STATIC_TESTABLE_DECL eSTATUS_t ActuatorsArm (void) {

    // #ifndef USE_SERVOS_ONLY

    uint32_t msDelay    = 2;
    uint32_t msMaxTime  = 350;
    uint32_t iterations = msMaxTime / msDelay;
    for (uint32_t i = 0; i < iterations; ++i) {
        /* NOTE: A DShot value of all 0s is a special command to
         * the esc to arm/disarm the motor depending on the esc's current state.
         * The reason MotorWrite isn't used is because it uses a valid throttle value between > 48 and < 2048 */
        for (uint32_t motorIdx = 0; motorIdx < eMOTOR_ID_MAX; ++motorIdx) {

            Motor* pMotor        = MotorGetById (motorIdx);
            eDEVICE_ID_t motorId = pMotor->id;
            if (MotorWriteCmd (motorId, eMOTOR_CMD_ARM) != eSTATUS_SUCCESS) {
                LOG_ERROR ("Failed to arm motor ID %u", motorId);
                return eSTATUS_FAILURE;
            }
        }
        // NOTE: assumes DShot150 is used.
        vTaskDelay (pdMS_TO_TICKS (msDelay));
    }

    // Slowly increase the throttle to 15%
    msDelay              = 4;
    msMaxTime            = 3000;
    float i              = MOTOR_MIN_THROTTLE;
    float targetThrottle = MOTOR_STARTUP_THROTTLE;
    float increment      = targetThrottle / (float)(msMaxTime / msDelay);
    while (i < targetThrottle) {
        for (uint32_t motorIdx = 0; motorIdx < eMOTOR_ID_MAX; ++motorIdx) {

            Motor* pMotor        = MotorGetById (motorIdx);
            eDEVICE_ID_t motorId = pMotor->id;
            if (MotorWrite (motorId, i) != eSTATUS_SUCCESS) {
                LOG_ERROR ("Failed to arm motor ID %u", motorId);
                return eSTATUS_FAILURE;
            }
        }
        vTaskDelay (pdMS_TO_TICKS (msDelay));
        i += increment;
    }

    return eSTATUS_SUCCESS;
}

eSTATUS_t ActuatorsInitMotor (MotorInitConf_t conf) {

    eSTATUS_t status           = MotorInit (conf);
    MotorBoardConf_t boardConf = conf.boardConf;
    ServoBoardConf_t* pLinkedServoBoardConf = boardConf.pLinkedServoBoardConf;
    eDEVICE_ID_t motorId = boardConf.motorId;
    if (status != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to initialize motor");
        return status;
    }

    Motor* pMotor = MotorGetById (motorId);
    if (pLinkedServoBoardConf != NULL) {
        eDEVICE_ID_t linkedServoId = pLinkedServoBoardConf->servoId;
        pMotor->linkedServoId      = linkedServoId;
    }
    return eSTATUS_SUCCESS;
}

eSTATUS_t ActuatorsInitServo (ServoInitConf_t conf) {

    eSTATUS_t status           = ServoInit (conf);
    ServoBoardConf_t boardConf = conf.boardConf;
    MotorBoardConf_t* pLinkedMotorBoardConf = boardConf.pLinkedMotorBoardConf;
    eDEVICE_ID_t servoId = boardConf.servoId;
    if (status != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to initialize servo");
        return status;
    }

    Servo* pServo = ServoGetById (servoId);
    if (pLinkedMotorBoardConf != NULL) {
        eDEVICE_ID_t linkedMotorId = pLinkedMotorBoardConf->motorId;
        pServo->linkedMotorId      = linkedMotorId;
    }
    return eSTATUS_SUCCESS;
}

// eSTATUS_t ActuatorsInit () {

//     eSTATUS_t status = ServoInit (eLEFT_SERVO_1_ID, left_ServoPWM,
//     &gLeftServo); if (status != eSTATUS_SUCCESS) {
//         LOG_ERROR ("Failed to initialize left servo");
//         return status;
//     }

//     status = MotorInit (eLEFT_MOTOR_ID, left_Motor, &gLeftMotor);
//     if (status != eSTATUS_SUCCESS) {
//         LOG_ERROR ("Failed to initialize left motor");
//         return status;
//     }

//     return eSTATUS_SUCCESS;
// }

eSTATUS_t ActuatorsStart (void) {

    // Loop over all motors and servos and start them

    // eSTATUS_t status = ServoStart (&gLeftServo);
    // if (status != eSTATUS_SUCCESS) {
    //     LOG_ERROR ("Failed to start left servo");
    //     return status;
    // }

    // status = MotorStart (&gLeftMotor);
    // if (status != eSTATUS_SUCCESS) {
    //     LOG_ERROR ("Failed to start left motor");
    //     return status;
    // }

    // status = ActuatorsArm ();
    // if (status != eSTATUS_SUCCESS) {
    //     LOG_ERROR ("Failed to arm actuators");
    //     return status;
    // }

    return eSTATUS_FAILURE;
}

eSTATUS_t ActuatorsStop (void) {

    // if (MotorStop (&gLeftMotor) != eSTATUS_SUCCESS) {
    //     LOG_ERROR ("Failed to stop left motor");
    //     return eSTATUS_FAILURE;
    // }

    // if (ServoStop (&gLeftServo) != eSTATUS_SUCCESS) {
    //     LOG_ERROR ("Failed to stop left servo");
    //     return eSTATUS_FAILURE;
    // }

    return eSTATUS_FAILURE;
}

eSTATUS_t ActuatorsWrite (Vec3f pidAttitude, float targetThrottle) {
    // TODO: write to motors and servos and mix linked motors and servos
    return eSTATUS_FAILURE;
    // return ActuatorsMixPair (&gLeftServo, &gLeftMotor, pidAttitude, targetThrottle);
}

Servo* ActuatorsGetLeftServo (void) {
    return &gLeftServo;
}

void ActuatorsLogData (void) {
    LOG_DATA_ACTUATORS_DATA ("Left Motor", gLeftMotor, "Left Servo", gLeftServo);
}