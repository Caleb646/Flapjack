#include "motion_control/actuators.h"
#include "flight_context.h"
#include "log.h"
#include <string.h>

#define CHECK_SERVO_DESCRIPTOR_OK(pServoDesc)                      \
    (                                                              \
    (pServoDesc) != NULL && (pServoDesc)->usLeftDutyCycle != 0U && \
    (pServoDesc)->usMiddleDutyCycle != 0U && (pServoDesc)->usRightDutyCycle != 0U)

#define CHECK_SERVO_OK(pServo) \
    ((pServo) != NULL && PWM_CHECK_OK (&((pServo)->pwmHandle)))


STATIC_TESTABLE_DECL float ServoAngle2PWM (ServoDescriptor* pServo, float targetAngle) {
    if (CHECK_SERVO_DESCRIPTOR_OK (pServo) != TRUE) {
        LOG_ERROR ("ServoDescriptor is not valid");
        return 0.0F;
    }

    // Handle asymmetric servo ranges: map negative angles to
    // [usLeftDutyCycle, usMiddleDutyCycle], positive to [usMiddleDutyCycle, usRightDutyCycle]
    if (targetAngle < 0) {
        return mapf32 (
        targetAngle, -pServo->maxAngle, 0.0F,
        (float)pServo->usLeftDutyCycle, (float)pServo->usMiddleDutyCycle);
    } else if (targetAngle > 0) {
        return mapf32 (
        targetAngle, 0.0F, pServo->maxAngle,
        (float)pServo->usMiddleDutyCycle, (float)pServo->usRightDutyCycle);
    }
    return (float)pServo->usMiddleDutyCycle;
}

STATUS_TYPE UpdateTargetAttitudeThrottle (
Vec3f maxAttitude,
RadioPWMChannels radio,
Vec3f* pOutputTargetAttitude,
float* pOutputThrottle) {

    *pOutputThrottle = clipf32 ((float)radio.channel1, 0.0F, 1.0F);
    pOutputTargetAttitude->roll =
    clipf32 ((float)radio.channel2, -1.0F, 1.0F) * maxAttitude.roll;

    pOutputTargetAttitude->pitch =
    clipf32 ((float)radio.channel3, -1.0F, 1.0F) * maxAttitude.pitch;

    pOutputTargetAttitude->yaw =
    clipf32 ((float)radio.channel4, -1.0F, 1.0F) * maxAttitude.yaw;

    return eSTATUS_SUCCESS;
}

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

STATUS_TYPE PIDInit (PIDContext* pContext) {
    memset ((void*)pContext, 0, sizeof (PIDContext));
    return eSTATUS_SUCCESS;
}

STATUS_TYPE ServoMove2Angle (Servo* pServo, float targetAngle) {
    if (CHECK_SERVO_OK (pServo) != TRUE) {
        LOG_ERROR ("Received invalid Servo pointer");
        return eSTATUS_FAILURE;
    }

    targetAngle = clipf32 (
    targetAngle, -pServo->pwmDescriptor.maxAngle, pServo->pwmDescriptor.maxAngle);

    return PWMSend (&pServo->pwmHandle, (uint32_t)ServoAngle2PWM (&pServo->pwmDescriptor, targetAngle));
}

// NOLINTBEGIN(cppcoreguidelines-avoid-non-const-global-variables)
static Motor gLeftMotor;
static Servo gLeftServo;
// NOLINTEND(cppcoreguidelines-avoid-non-const-global-variables)

// static Motor rightMotor;
// static Servo rightServo;

STATUS_TYPE TestServoMove2Angle (float targetAngle) {
    return ServoMove2Angle (&gLeftServo, targetAngle);
}

/*
 * \param pidAttitude roll, pitch, and yaw are between -1 and 1
 */
STATUS_TYPE PWMMixPIDnSend (Vec3f pidAttitude, float targetThrottle) {
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
    float target        = 0.0F;
    MotorDescriptor* pM = &gLeftMotor.pwmDescriptor;
    target =
    targetThrottle - pidAttitude.pitch + pidAttitude.roll + pidAttitude.yaw;
    uint32_t usMotorTargetDutyCycle = (uint32_t)mapf32 (
    target, -3.0F, 4.0F, (float)pM->usMinDutyCycle, (float)pM->usMaxDutyCycle);

    /*
     *  Left Servo Mixing
     */
    ServoDescriptor* pSer = &gLeftServo.pwmDescriptor;
    target                = pSer->pitchMix * pidAttitude.pitch +
             pSer->rollMix * pidAttitude.roll + pSer->yawMix * pidAttitude.yaw;
    // NOTE: Maybe Roll should have a negative impact on target angle. Meaning the magnitude of the target angle is closer to 0
    // the larger pid roll is.
    // float target = pSer->pitchMix * pidAttitude.pitch + pSer->yawMix * pidAttitude.yaw;
    // pSer->targetAngle = ( clipf32(target, -1.0f, 1.0f) * pSer->maxAngle ) / ( clipf32(pSer->rollMix * pidAttitude.roll, -1.0f, 1.0f) * pSer->maxAngle );
    target         = clipf32 (target, -1.0F, 1.0F) * pSer->maxAngle;
    pSer->curAngle = target;
    uint32_t usServoTargetDutyCycle = (uint32_t)ServoAngle2PWM (pSer, target);

    PWMSend (&gLeftMotor.pwmHandle, usMotorTargetDutyCycle);
    PWMSend (&gLeftServo.pwmHandle, usServoTargetDutyCycle);

    return eSTATUS_SUCCESS;
}

STATUS_TYPE ActuatorsInit (PWMHandle leftMotorPWM, PWMHandle leftServoPWM) {
    // // Prescale 64MHz clock to 1MHz
    // PWM_SET_PRESCALER (&leftMotorPWM, 64);
    // // Use ARR register to scale clock from 1MHz to 4000Hz (250us)
    // PWM_SET_PERIOD (&leftMotorPWM, 250);
    // // Set duty cycle to 0 percent
    // PWM_SET_COMPARE (&leftMotorPWM, 0);

    // // Prescale 64MHz clock to 1MHz
    // PWM_SET_PRESCALER (&leftServoPWM, 64);
    // // Use ARR register to scale clock from 1MHz to 50Hz
    // // 1MHz / 20,000 = 50Hz
    // PWM_SET_PERIOD (&leftServoPWM, 20000);
    // // Set duty cycle to 0 percent
    // PWM_SET_COMPARE (&leftServoPWM, 0);
    if (PWMInit (&leftMotorPWM) != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to init left motor PWM");
        return eSTATUS_FAILURE;
    }
    if (PWMInit (&leftServoPWM) != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to init left servo PWM");
        return eSTATUS_FAILURE;
    }

    if (PWMStart (&leftMotorPWM) != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to start left motor PWM");
        return eSTATUS_FAILURE;
    }

    if (PWMStart (&leftServoPWM) != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to start left servo PWM");
        return eSTATUS_FAILURE;
    }

    memset ((void*)&gLeftMotor, 0, sizeof (Motor));
    memset ((void*)&gLeftServo, 0, sizeof (Servo));
    // memset((void*)&rightMotor, 0, sizeof(Motor));
    // memset((void*)&rightServo, 0, sizeof(Servo));

    MotorDescriptor motorDescriptor;
    memset ((void*)&motorDescriptor, 0, sizeof (MotorDescriptor));
    motorDescriptor.usMinDutyCycle = 125;
    motorDescriptor.usMaxDutyCycle = 250;

    gLeftMotor.pwmDescriptor = motorDescriptor;
    gLeftMotor.pwmHandle     = leftMotorPWM;

    ServoDescriptor servoDescriptor;
    memset ((void*)&servoDescriptor, 0, sizeof (ServoDescriptor));
    servoDescriptor.usLeftDutyCycle   = 1000;
    servoDescriptor.usMiddleDutyCycle = 1500;
    servoDescriptor.usRightDutyCycle  = 2000;

    servoDescriptor.maxAngle = 20;
    servoDescriptor.curAngle = 0;

    servoDescriptor.rollMix  = -0.25F;
    servoDescriptor.yawMix   = 0.5F;
    servoDescriptor.pitchMix = 0.5F;


    gLeftServo.pwmDescriptor = servoDescriptor;
    gLeftServo.pwmHandle     = leftServoPWM;

    return eSTATUS_SUCCESS;
}