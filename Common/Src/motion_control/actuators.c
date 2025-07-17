#include "motion_control/actuators.h"
#include "flight_context.h"
#include "log.h"
#include "motion_control/dshot.h"
#include "motion_control/pwm.h"
#include <string.h>


#define CHECK_SERVO_DESCRIPTOR_OK(pServoDesc)                      \
    (                                                              \
    (pServoDesc) != NULL && (pServoDesc)->usLeftDutyCycle != 0U && \
    (pServoDesc)->usMiddleDutyCycle != 0U && (pServoDesc)->usRightDutyCycle != 0U)

#define CHECK_SERVO_OK(pServo) \
    ((pServo) != NULL && PWM_CHECK_OK (&((pServo)->pwm)))

// STATUS_TYPE RadioUpdateTargetAttitudeThrottle (
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

STATUS_TYPE PIDUpdateAttitude (
PIDContext* pidContext,
Vec3f currentAttitude, // degrees
Vec3f targetAttitude,  // degrees
Vec3f maxAttitude,     // degrees
float dt,
Vec3f* pOutputPIDAttitude // degrees
) {
    float P            = 0.0F;
    float I            = 0.0F;
    float D            = 0.0F;
    P                  = pidContext->rollP;
    I                  = pidContext->rollI;
    D                  = pidContext->rollD;
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

// STATUS_TYPE PIDInit (PIDContext* pContext) {
//     memset ((void*)pContext, 0, sizeof (PIDContext));
//     return eSTATUS_SUCCESS;
// }

STATIC_TESTABLE_DECL float ServoAngle2PWM (Servo* pServo, float targetAngle) {
    if (CHECK_SERVO_OK (pServo) != TRUE) {
        LOG_ERROR ("ServoDescriptor is not valid");
        return 0.0F;
    }

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

STATUS_TYPE ServoInit (PWMConfig config, Servo* pOutServo) {
    if (pOutServo == NULL) {
        LOG_ERROR ("Received NULL pointer for Servo");
        return eSTATUS_FAILURE;
    }

    memset ((void*)pOutServo, 0, sizeof (Servo));
    Servo servo = { 0 };
    if (PWMInit (config, &servo.pwm) != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to initialize PWM for Servo");
        return eSTATUS_FAILURE;
    }

    ServoDescriptor servoDescriptor   = { 0 };
    servoDescriptor.usLeftDutyCycle   = 1000;
    servoDescriptor.usMiddleDutyCycle = 1500;
    servoDescriptor.usRightDutyCycle  = 2000;
    servoDescriptor.maxAngle          = 20;
    servoDescriptor.curAngle          = 0;
    servoDescriptor.rollMix           = -0.25F;
    servoDescriptor.yawMix            = 0.5F;
    servoDescriptor.pitchMix          = 0.5F;

    servo.desc = servoDescriptor;
    *pOutServo = servo;

    return eSTATUS_SUCCESS;
}

STATUS_TYPE ServoStart (Servo* pServo) {
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

STATUS_TYPE ServoWrite (Servo* pServo, float targetAngle) {
    if (CHECK_SERVO_OK (pServo) != TRUE) {
        LOG_ERROR ("Received invalid Servo pointer");
        return eSTATUS_FAILURE;
    }

    targetAngle =
    clipf32 (targetAngle, -pServo->desc.maxAngle, pServo->desc.maxAngle);
    return PWMWrite (&pServo->pwm, (uint32_t)ServoAngle2PWM (pServo, targetAngle));
}

STATUS_TYPE MotorInit (MotorConfig config, Motor* pOutMotor) {
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
    return eSTATUS_SUCCESS;
}

STATUS_TYPE MotorStart (Motor* pMotor) {
    if (pMotor == NULL) {
        LOG_ERROR ("Received NULL pointer for Motor");
        return eSTATUS_FAILURE;
    }

    if (DShotStart (&pMotor->dshot) != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to start DShot for Motor");
        return eSTATUS_FAILURE;
    }

    // Set initial throttle to minimum
    return DShotWrite (&pMotor->dshot, DSHOT_MIN_THROTTLE);
}

/*
 * throttle is between 0.0F and 1.0F
 */
STATUS_TYPE MotorWrite (Motor* pMotor, float throttle) {
    if (pMotor == NULL) {
        LOG_ERROR ("Received NULL pointer for Motor");
        return eSTATUS_FAILURE;
    }

    if (throttle < 0.0F || throttle > 1.0F) {
        LOG_ERROR ("Motor value out of range: %u", (uint16_t)(throttle * 100.0F));
        return eSTATUS_FAILURE;
    }

    return DShotWrite (&pMotor->dshot, DSHOT_MIN_THROTTLE + (uint16_t)(throttle * DSHOT_RANGE));
}

// NOLINTBEGIN(cppcoreguidelines-avoid-non-const-global-variables)
static Motor gLeftMotor;
static Servo gLeftServo;
// NOLINTEND(cppcoreguidelines-avoid-non-const-global-variables)

// static Motor rightMotor;
// static Servo rightServo;

/*
 * \param pidAttitude roll, pitch, and yaw are between -1 and 1
 */
STATIC_TESTABLE_DECL STATUS_TYPE
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
     *  Left Motor Mixing
     */
    // float target        = 0.0F;
    // MotorDescriptor* pM = &gLeftMotor.pwmDescriptor;
    // target =
    // targetThrottle - pidAttitude.pitch + pidAttitude.roll + pidAttitude.yaw;
    // uint32_t usMotorTargetDutyCycle = (uint32_t)mapf32 (
    // target, -3.0F, 4.0F, (float)pM->usMinDutyCycle, (float)pM->usMaxDutyCycle);

    /*
     *  Left Servo Mixing
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
    pServoDesc->curAngle = target;

    /* Motor throttle should be between 0 and 1 */
    MotorWrite (pMotor, targetThrottle);
    ServoWrite (pServo, target);
    return eSTATUS_SUCCESS;
}

STATUS_TYPE ActuatorsInit (PWMConfig left_ServoPWM, MotorConfig left_Motor) {

    STATUS_TYPE status = ServoInit (left_ServoPWM, &gLeftServo);
    if (status != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to initialize left servo");
        return status;
    }

    status = MotorInit (left_Motor, &gLeftMotor);
    if (status != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to initialize left motor");
        return status;
    }

    return eSTATUS_SUCCESS;
}

STATUS_TYPE ActuatorsStart (void) {

    STATUS_TYPE status = ServoStart (&gLeftServo);
    if (status != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to start left servo");
        return status;
    }

    status = MotorStart (&gLeftMotor);
    if (status != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to start left motor");
        return status;
    }

    return eSTATUS_SUCCESS;
}

STATUS_TYPE ActuatorsWrite (Vec3f pidAttitude, float targetThrottle) {
    // if (
    // pidAttitude.roll < -1.0F || pidAttitude.roll > 1.0F ||
    // pidAttitude.pitch < -1.0F || pidAttitude.pitch > 1.0F ||
    // pidAttitude.yaw < -1.0F || pidAttitude.yaw > 1.0F) {
    //     LOG_ERROR (
    //     "PID attitude values out of range: roll: %f, pitch: %f, yaw: %f",
    //     pidAttitude.roll, pidAttitude.pitch, pidAttitude.yaw);
    //     return eSTATUS_FAILURE;
    // }

    return ActuatorsMixPair (&gLeftServo, &gLeftMotor, pidAttitude, targetThrottle);
}