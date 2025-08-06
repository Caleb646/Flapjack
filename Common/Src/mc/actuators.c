#include "mc/actuators.h"
#include "conf.h"
#include "log.h"
#include "mc/dshot.h"
#include "mc/pwm.h"
#include <string.h>


#define CHECK_SERVO_DESCRIPTOR_OK(pServoDesc)                      \
    (                                                              \
    (pServoDesc) != NULL && (pServoDesc)->usLeftDutyCycle != 0U && \
    (pServoDesc)->usMiddleDutyCycle != 0U && (pServoDesc)->usRightDutyCycle != 0U)

#define CHECK_SERVO_OK(pServo) \
    ((pServo) != NULL && PWM_CHECK_OK (&((pServo)->pwm)))

// eSTATUS_t RadioUpdateTargetAttitudeThrottle (
// Vec3f maxAttitude,
// RadioPWMChannels radio,
// Vec3f* pOutputTargetAttitude,
// float* pOutputThrottle) {

//     *pOutputThrottle = clipf32 ((float)radio.channel1, 0.0F, 1.0F);
//     pOutputTargetAttitude->roll =
//     clipf32 ((float)radio.channel2, -1.0F, 1.0F) * maxAttitude.roll;

//     pOutputTargetAttitude->pitch =
//     clipf32 ((float)radio.channel3, -1.0F, 1.0F) * maxAttitude.pitch;

//     pOutputTargetAttitude->yaw =
//     clipf32 ((float)radio.channel4, -1.0F, 1.0F) * maxAttitude.yaw;

//     return eSTATUS_SUCCESS;
// }

eSTATUS_t PIDUpdateAttitude (
PIDContext* pidContext,
Vec3f currentAttitude, // degrees
Vec3f targetAttitude,  // degrees
Vec3f maxAttitude,     // degrees
float dt,
Vec3f* pOutputPIDAttitude // degrees
) {

    if (pidContext == NULL || pOutputPIDAttitude == NULL) {
        LOG_ERROR ("PIDContext or output pointer is NULL");
        return eSTATUS_FAILURE;
    }

    float P            = pidContext->rollP;
    float I            = pidContext->rollI;
    float D            = pidContext->rollD;
    float rollError    = targetAttitude.roll - currentAttitude.roll;
    float rollIntegral = clipf32 (
    pidContext->prevIntegral.roll + rollError * dt,
    -pidContext->integralLimit, pidContext->integralLimit);
    float rollDerivative =
    (dt != 0.0F) ? (rollError - pidContext->prevError.roll) / dt : 0.0F;
    // pOutputPIDAttitude->roll = 0.01f * (P * rollError + I * rollIntegral - D * rollDerivative);

    // Scale PID output between -1 and 1
    pOutputPIDAttitude->roll = clipf32 (
                               (P * rollError + I * rollIntegral - D * rollDerivative),
                               -maxAttitude.roll, maxAttitude.roll) /
                               maxAttitude.roll;

    P                   = pidContext->pitchP;
    I                   = pidContext->pitchI;
    D                   = pidContext->pitchD;
    float pitchError    = targetAttitude.pitch - currentAttitude.pitch;
    float pitchIntegral = clipf32 (
    pidContext->prevIntegral.pitch + pitchError * dt,
    -pidContext->integralLimit, pidContext->integralLimit);
    float pitchDerivative =
    (dt != 0.0F) ? (pitchError - pidContext->prevError.pitch) / dt : 0.0F;
    // pOutputPIDAttitude->pitch = 0.01f * (P * pitchError + I * pitchIntegral - D * pitchDerivative);

    // Scale PID output between -1 and 1
    pOutputPIDAttitude->pitch =
    clipf32 (
    (P * pitchError + I * pitchIntegral - D * pitchDerivative),
    -maxAttitude.pitch, maxAttitude.pitch) /
    maxAttitude.pitch;

    P                 = pidContext->yawP;
    I                 = pidContext->yawI;
    D                 = pidContext->yawD;
    float yawError    = targetAttitude.yaw - currentAttitude.yaw;
    float yawIntegral = clipf32 (
    pidContext->prevIntegral.yaw + yawError * dt,
    -pidContext->integralLimit, pidContext->integralLimit);
    float yawDerivative =
    (dt != 0.0F) ? (yawError - pidContext->prevError.yaw) / dt : 0.0F;
    // pOutputPIDAttitude->yaw = 0.01f * (P * yawError + I * yawIntegral - D * yawDerivative);

    // Scale PID output between -1 and 1
    pOutputPIDAttitude->yaw = clipf32 (
                              (P * yawError + I * yawIntegral - D * yawDerivative),
                              -maxAttitude.yaw, maxAttitude.yaw) /
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

STATIC_TESTABLE_DECL float ServoAngle2PWM (Servo* pServo, float targetAngle) {
    if (CHECK_SERVO_OK (pServo) != TRUE) {
        LOG_ERROR ("ServoDescriptor is not valid");
        return 0.0F;
    }
    /*
     * NOTE: The servo on its own can move between -maxAngle and +maxAngle. But when placed
     * in the drone, it may only be able to move between -usableMaxAngle and +usableMaxAngle.
     *
     * Clip the target angle to the usable range first, then map it to the PWM duty cycle using the max angle.
     */
    ServoDescriptor* pDesc = &pServo->desc;
    // Handle asymmetric servo ranges: map negative angles to
    // [usLeftDutyCycle, usMiddleDutyCycle], positive to [usMiddleDutyCycle, usRightDutyCycle]
    if (targetAngle < 0) {
        return mapf32 (
        targetAngle, -pDesc->maxAngle, 0.0F, (float)pDesc->usLeftDutyCycle,
        (float)pDesc->usMiddleDutyCycle);
    }

    if (targetAngle > 0) {
        return mapf32 (
        targetAngle, 0.0F, pDesc->maxAngle,
        (float)pDesc->usMiddleDutyCycle, (float)pDesc->usRightDutyCycle);
    }

    return (float)pDesc->usMiddleDutyCycle;
}

eSTATUS_t ServoInit (PWMConfig config, Servo* pOutServo) {
    if (pOutServo == NULL) {
        LOG_ERROR ("Received NULL pointer for Servo");
        return eSTATUS_FAILURE;
    }

    memset ((void*)pOutServo, 0, sizeof (Servo));
    if (PWMInit (config, &pOutServo->pwm) != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to initialize PWM for Servo");
        return eSTATUS_FAILURE;
    }

    // servoDescriptor.usLeftDutyCycle   = 1000;
    // servoDescriptor.usMiddleDutyCycle = 1500;
    // servoDescriptor.usRightDutyCycle  = 2000;
    pOutServo->desc.usLeftDutyCycle   = 550;
    pOutServo->desc.usMiddleDutyCycle = 1600;
    pOutServo->desc.usRightDutyCycle  = 2650;
    pOutServo->desc.maxAngle          = 90.0F;
    pOutServo->desc.usableMaxAngle    = 20.0F;
    pOutServo->desc.curTargetAngle    = 0.0F;
    // TODO: Make these configurable
    pOutServo->desc.rollMix  = LEFT_SERVO_ROLL_MIX;  // -0.25F;
    pOutServo->desc.yawMix   = LEFT_SERVO_YAW_MIX;   // 0.5F;
    pOutServo->desc.pitchMix = LEFT_SERVO_PITCH_MIX; // 0.5F;

    return eSTATUS_SUCCESS;
}

eSTATUS_t ServoStart (Servo* pServo) {
    if (CHECK_SERVO_OK (pServo) != TRUE) {
        LOG_ERROR ("Received invalid Servo pointer");
        return eSTATUS_FAILURE;
    }

    if (PWMStart (&pServo->pwm) != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to start PWM for Servo");
        return eSTATUS_FAILURE;
    }

    // Set initial angle to middle position
    return ServoWrite (pServo, 0.0F);
}

eSTATUS_t ServoStop (Servo* pServo) {
    if (CHECK_SERVO_OK (pServo) != TRUE) {
        LOG_ERROR ("Received invalid Servo pointer");
        return eSTATUS_FAILURE;
    }

    if (PWMStop (&pServo->pwm) != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to stop PWM for Servo");
        return eSTATUS_FAILURE;
    }

    return eSTATUS_SUCCESS;
}

eSTATUS_t ServoWrite (Servo* pServo, float targetAngle) {
    if (CHECK_SERVO_OK (pServo) != TRUE) {
        LOG_ERROR ("Received invalid Servo pointer");
        return eSTATUS_FAILURE;
    }
    targetAngle =
    clipf32 (targetAngle, -pServo->desc.usableMaxAngle, pServo->desc.usableMaxAngle);
    pServo->desc.curAngle = targetAngle;

    return PWMWrite (&pServo->pwm, (uint32_t)ServoAngle2PWM (pServo, targetAngle));
}


#ifndef UNIT_TEST

#endif // UNIT_TEST

eSTATUS_t MotorInit (MotorConfig config, Motor* pOutMotor) {

#ifndef USE_SERVOS_ONLY

    if (pOutMotor == NULL) {
        LOG_ERROR ("Received NULL pointer for Motor");
        return eSTATUS_FAILURE;
    }

    memset ((void*)pOutMotor, 0, sizeof (Motor));
    DShotConfig dshotConfig = { 0 };
    dshotConfig.dshotType   = DSHOT150; // Set DShot frequency

    if (DShotInit (dshotConfig, config.pwm, config.dma, &pOutMotor->dshot) != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to initialize DShot for Motor");
        return eSTATUS_FAILURE;
    }

    MotorDescriptor motorDescriptor = { 0 };
    // motorDescriptor.usMinDutyCycle  = 50;
    // motorDescriptor.usMaxDutyCycle  = 2000;

    pOutMotor->desc = motorDescriptor;

#endif

    return eSTATUS_SUCCESS;
}

eSTATUS_t MotorStart (Motor* pMotor) {

#ifndef USE_SERVOS_ONLY

    if (pMotor == NULL) {
        LOG_ERROR ("Received NULL pointer for Motor");
        return eSTATUS_FAILURE;
    }

    if (DShotStart (&pMotor->dshot) != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to start DShot for Motor");
        return eSTATUS_FAILURE;
    }

#endif

    // Set initial throttle to minimum
    // return DShotWrite (&pMotor->dshot, DSHOT_MIN_THROTTLE);
    return eSTATUS_SUCCESS;
}

eSTATUS_t MotorStop (Motor* pMotor) {

#ifndef USE_SERVOS_ONLY

    if (pMotor == NULL) {
        LOG_ERROR ("Received NULL pointer for Motor");
        return eSTATUS_FAILURE;
    }

    /*
     * NOTE: The motor's ESC will automatically stop the motor
     * when the PWM signal has stopped for more than 5-10ms.
     */
    if (DShotStop (&pMotor->dshot) != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to stop dshot");
        return eSTATUS_FAILURE;
    }
#endif

    return eSTATUS_SUCCESS;
}

/*
 * throttle is between 0.0F and 1.0F
 */
eSTATUS_t MotorWrite (Motor* pMotor, float throttle) {

#ifndef USE_SERVOS_ONLY

    if (pMotor == NULL) {
        LOG_ERROR ("Received NULL pointer for Motor");
        return eSTATUS_FAILURE;
    }

    if (throttle < 0.0F || throttle > 1.0F) {
        LOG_ERROR ("Motor throttle out of range: %u", (uint16_t)(throttle * 100.0F));
        return eSTATUS_FAILURE;
    }

    throttle = clipf32 (throttle, MOTOR_MIN_THROTTLE, MOTOR_MAX_THROTTLE);
    pMotor->desc.curThrottle = throttle;

    eSTATUS_t status =
    DShotWrite (&pMotor->dshot, DSHOT_MIN_THROTTLE + (uint16_t)(throttle * (float)DSHOT_RANGE));
    if (status != eSTATUS_SUCCESS && status != eSTATUS_BUSY) {
        LOG_ERROR ("Failed to write to motor");
        return status;
    }

#endif
    /*
     * NOTE: DShotWrite returns eSTATUS_BUSY when a write is in progress.
     * Don't consider this a failure.
     */
    return eSTATUS_SUCCESS;
}

eSTATUS_t MotorWriteCmd (Motor* pMotor, eMOTOR_CMD_t command) {

#ifndef USE_SERVOS_ONLY

    if (pMotor == NULL) {
        LOG_ERROR ("Received NULL pointer for Motor");
        return eSTATUS_FAILURE;
    }

    switch (command) {
    // NOTE: The arm and disarm command values are the same and depend on the ESC's current state.
    case eMOTOR_CMD_ARM: break;
    // case eMOTOR_CMD_DISARM: break;
    default: LOG_ERROR ("Unknown motor command"); return eSTATUS_FAILURE;
    }

    eSTATUS_t status = DShotWrite (&pMotor->dshot, command);
    if (status != eSTATUS_SUCCESS && status != eSTATUS_BUSY) {
        LOG_ERROR ("Failed to write command to motor");
        return status;
    }

#endif // USE_SERVOS_ONLY

    return eSTATUS_SUCCESS;
}

// NOLINTBEGIN(cppcoreguidelines-avoid-non-const-global-variables)
static Motor gLeftMotor;
static Servo gLeftServo;
// NOLINTEND(cppcoreguidelines-avoid-non-const-global-variables)

// static Motor rightMotor;
// static Servo rightServo;

#ifndef UNIT_TEST

static eSTATUS_t
ActuatorsMixPair (Servo* pServo, Motor* pMotor, Vec3f pidAttitude, float targetThrottle);
static eSTATUS_t ActuatorsArm (void);

#endif // UNIT_TEST

/*
 * \param pidAttitude roll, pitch, and yaw are between -1 and 1
 */
STATIC_TESTABLE_DECL eSTATUS_t
ActuatorsMixPair (Servo* pServo, Motor* pMotor, Vec3f pidAttitude, float targetThrottle) {
    /*
     *   NWU coordinate system:
     *       + x-axis pointing to the North
     *       + y-axis pointing to the West
     *       + z-axis pointing away from the surface of the Earth
     *   + roll (pivot on x axis) right motor is higher
     *   + pitch (pivot on y axis) nose is higher than tail
     *   + yaw (pivot on z axis) right wing tip is more forward
     */

    /*
     *  Motor Mixing
     */
    // float target        = 0.0F;
    // MotorDescriptor* pM = &gLeftMotor.pwmDescriptor;
    // target =
    // targetThrottle - pidAttitude.pitch + pidAttitude.roll + pidAttitude.yaw;
    // uint32_t usMotorTargetDutyCycle = (uint32_t)mapf32 (
    // target, -3.0F, 4.0F, (float)pM->usMinDutyCycle, (float)pM->usMaxDutyCycle);
    pMotor->desc.curTargetThrottle = targetThrottle;

    /*
     *  Servo Mixing
     */
    ServoDescriptor* pServoDesc = &gLeftServo.desc;
    float target = pServoDesc->pitchMix * pidAttitude.pitch +
                   pServoDesc->rollMix * pidAttitude.roll +
                   pServoDesc->yawMix * pidAttitude.yaw;
    // NOTE: Maybe Roll should have a negative impact on target angle. Meaning the magnitude of the target angle is closer to 0
    // the larger pid roll is.
    // float target = pSer->pitchMix * pidAttitude.pitch + pSer->yawMix * pidAttitude.yaw;
    // pSer->targetAngle = ( clipf32(target, -1.0f, 1.0f) * pSer->maxAngle ) / ( clipf32(pSer->rollMix * pidAttitude.roll, -1.0f, 1.0f) * pSer->maxAngle );
    target = clipf32 (target, -1.0F, 1.0F) * pServoDesc->maxAngle;
    pServoDesc->curTargetAngle = target;

    // #ifndef USE_SERVOS_ONLY
    /* Motor throttle should be between 0 and 1 */
    MotorWrite (pMotor, targetThrottle);
    // #endif
    ServoWrite (pServo, target);
    return eSTATUS_SUCCESS;
}

/*
 * \brief After motors are armed, a motor write has to be issued at
 * least every 5ms or the motor ESC will stop the motor.
 */
STATIC_TESTABLE_DECL eSTATUS_t ActuatorsArm (void) {

    // #ifndef USE_SERVOS_ONLY

    uint16_t msDelay    = 2;
    uint16_t msMaxTime  = 350;
    uint16_t iterations = msMaxTime / msDelay;
    for (uint32_t i = 0; i < iterations; ++i) {
        /* NOTE: A DShot value of all 0s is a special command to
         * the esc to arm/disarm the motor depending on the esc's current state.
         * The reason MotorWrite isn't used is because it uses a valid throttle value between > 48 and < 2048 */
        if (MotorWriteCmd (&gLeftMotor, eMOTOR_CMD_ARM) != eSTATUS_SUCCESS) {
            LOG_ERROR ("Failed to arm left motor");
            return eSTATUS_FAILURE;
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
        if (MotorWrite (&gLeftMotor, i) != eSTATUS_SUCCESS) {
            LOG_ERROR ("Failed warm up left motor");
            return eSTATUS_FAILURE;
        }
        vTaskDelay (pdMS_TO_TICKS (msDelay));
        i += increment;
    }

    // #endif

    return eSTATUS_SUCCESS;
}

eSTATUS_t ActuatorsInit (PWMConfig left_ServoPWM, MotorConfig left_Motor) {

    eSTATUS_t status = ServoInit (left_ServoPWM, &gLeftServo);
    if (status != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to initialize left servo");
        return status;
    }

    // #ifndef USE_SERVOS_ONLY

    status = MotorInit (left_Motor, &gLeftMotor);
    if (status != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to initialize left motor");
        return status;
    }

    // #endif
    return eSTATUS_SUCCESS;
}

eSTATUS_t ActuatorsStart (void) {

    eSTATUS_t status = ServoStart (&gLeftServo);
    if (status != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to start left servo");
        return status;
    }

    // #ifndef USE_SERVOS_ONLY

    status = MotorStart (&gLeftMotor);
    if (status != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to start left motor");
        return status;
    }

    status = ActuatorsArm ();
    if (status != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to arm actuators");
        return status;
    }

    // #endif

    return eSTATUS_SUCCESS;
}

eSTATUS_t ActuatorsStop (void) {

#ifndef USE_SERVOS_ONLY
    if (MotorStop (&gLeftMotor) != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to stop left motor");
        return eSTATUS_FAILURE;
    }
#endif

    if (ServoStop (&gLeftServo) != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to stop left servo");
        return eSTATUS_FAILURE;
    }

    return eSTATUS_SUCCESS;
}

eSTATUS_t ActuatorsWrite (Vec3f pidAttitude, float targetThrottle) {
    return ActuatorsMixPair (&gLeftServo, &gLeftMotor, pidAttitude, targetThrottle);
}

Servo* ActuatorsGetLeftServo (void) {
    return &gLeftServo;
}

void ActuatorsLogData (void) {
    LOG_DATA_ACTUATORS_DATA ("Left Motor", gLeftMotor, "Left Servo", gLeftServo);
}