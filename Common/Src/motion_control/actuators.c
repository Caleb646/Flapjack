#include <string.h>
#include "motion_control/actuators.h"
#include "flight_context.h"

STATUS_TYPE UpdateTargetAttitudeThrottle(
    FlightContext *pFlightContext, 
    RadioPWMChannels radio, 
    Vec3f *pOutputTargetAttitude, 
    float *pOutputThrottle
)
{
    *pOutputThrottle = clipf32((float)radio.channel1, 0.0f, 1.0f);

    pOutputTargetAttitude->roll = clipf32(
        (float)radio.channel2, -1.0f, 1.0f
    ) * pFlightContext->maxAttitude.roll;

    pOutputTargetAttitude->pitch = clipf32(
        (float)radio.channel3, -1.0f, 1.0f
    ) * pFlightContext->maxAttitude.pitch;

    pOutputTargetAttitude->yaw = clipf32(
        (float)radio.channel4, -1.0f, 1.0f
    ) * pFlightContext->maxAttitude.yaw;

    return eSTATUS_SUCCESS;
}

STATUS_TYPE PIDUpdateAttitude(
    PIDContext *pidContext,
    Vec3f imuGyro, // degrees per second
    Vec3f currentAttitude, // degrees
    Vec3f targetAttitude, // degrees
    float dt, 
    Vec3f *pOutputPIDAttitude // degrees
)
{
    float P, I, D;

    P = pidContext->rollP;
    I = pidContext->rollI;
    D = pidContext->rollD;
    float rollError = targetAttitude.roll - currentAttitude.roll;
    float rollIntegral = clipf32(
        pidContext->prevIntegral.roll + rollError * dt, 
        -pidContext->integralLimit, 
        pidContext->integralLimit
    );
    float rollDerivative = imuGyro.x;
    pOutputPIDAttitude->roll = 0.01f * (P * rollError + I * rollIntegral - D * rollDerivative);

    P = pidContext->pitchP;
    I = pidContext->pitchI;
    D = pidContext->pitchD;
    float pitchError = targetAttitude.pitch - currentAttitude.pitch;
    float pitchIntegral = clipf32(
        pidContext->prevIntegral.pitch + pitchError * dt, 
        -pidContext->integralLimit, 
        pidContext->integralLimit
    );
    float pitchDerivative = imuGyro.y;
    pOutputPIDAttitude->pitch = 0.01f * (P * pitchError + I * pitchIntegral - D * pitchDerivative);

    P = pidContext->yawP;
    I = pidContext->yawI;
    D = pidContext->yawD;
    float yawError = targetAttitude.yaw - currentAttitude.yaw;
    float yawIntegral = clipf32(
        pidContext->prevIntegral.yaw + yawError * dt, 
        -pidContext->integralLimit, 
        pidContext->integralLimit
    );
    float yawDerivative = imuGyro.z;
    pOutputPIDAttitude->yaw = 0.01f * (P * yawError + I * yawIntegral - D * yawDerivative);

    pidContext->prevIntegral.roll = rollIntegral;
    pidContext->prevIntegral.pitch = pitchIntegral;
    pidContext->prevIntegral.yaw = yawIntegral;

    pidContext->prevError.roll = rollError;
    pidContext->prevError.pitch = pitchError;
    pidContext->prevError.yaw = yawError;

    return eSTATUS_SUCCESS;
}

STATUS_TYPE PIDInit(PIDContext *pContext)
{
    memset((void*)pContext, 0, sizeof(PIDContext));
    return eSTATUS_SUCCESS;
}

/*
* PWM & Motion Control
*/
static Motor leftMotor;
static Servo leftServo;
// static Motor rightMotor;
// static Servo rightServo;
STATUS_TYPE PID2PWMMixer(Vec3f pidAttitude, float targetThrottle)
{
    leftMotor.pwmDescriptor.scaledDutyCycle = targetThrottle - pidAttitude.pitch + pidAttitude.roll + pidAttitude.yaw;

    return eSTATUS_SUCCESS;
}

STATUS_TYPE MotionControlInit(
    PWMHandle leftMotorInter, PWMHandle leftServoInter
)
{
    memset((void*)&leftMotor, 0, sizeof(Motor));
    memset((void*)&leftServo, 0, sizeof(Servo));
    // memset((void*)&rightMotor, 0, sizeof(Motor));
    // memset((void*)&rightServo, 0, sizeof(Servo));

    PWMDescriptor motorDescriptor;
    motorDescriptor.usMinDutyCycle = 125;
    motorDescriptor.usMaxDutyCycle = 250;
    motorDescriptor.usMaxPWMCycle = 250;

    PWMDescriptor servoDescriptor;
    servoDescriptor.usMinDutyCycle = 1000;
    servoDescriptor.usMaxDutyCycle = 2000;
    servoDescriptor.usMaxPWMCycle = 20000;

    // rightMotor.pwmDescriptor = motorDescriptor;
    // rightServo.pwmDescriptor = servoDescriptor;

    // PWMInterfaceDescriptor leftMotorInter;
    // leftMotorInter.pTimerHandle = pLeftMotorHandle;
    // leftMotorInter.pTimerRegister = pLeftMotorRegister;

    // PWMInterfaceDescriptor leftServoInter;
    // leftServoInter.pTimerHandle = pLeftServoHandle;
    // leftServoInter.pTimerRegister = pLeftServoRegister;

    // PWMInterfaceDescriptor rightMotorInter;
    // PWMInterfaceDescriptor rightServoInter;

    leftMotor.pwmDescriptor = motorDescriptor;
    leftMotor.pwmInterface = leftMotorInter;

    leftServo.pwmDescriptor = servoDescriptor;
    leftServo.pwmInterface = leftServoInter;
    leftServo.minAngle = -90;
    leftServo.maxAngle = 90;
    leftServo.curAngle = 0;

    return eSTATUS_SUCCESS;
}

// void MotionControlUpdatePWM(
//     AxisMap axisMap, Vec3 velMax, Vec3 velAngMax, Vec3 mmVelSteps, Vec3 mmAngVelSteps, void *devs, uint32_t nDevs
// )
// {
//     int32_t forward = mmVelSteps.x * axisMap.forward.x + mmVelSteps.y * axisMap.forward.y + mmVelSteps.z * axisMap.forward.z;
//     int32_t up = mmVelSteps.x * axisMap.up.x + mmVelSteps.y * axisMap.up.y + mmVelSteps.z * axisMap.up.z;
//     int32_t right = mmVelSteps.x * axisMap.right.x + mmVelSteps.y * axisMap.right.y + mmVelSteps.z * axisMap.right.z;

//     // Left wing tip is higher than right wing tip
//     int32_t rollRight = mmAngVelSteps.x * axisMap.forward.x + mmAngVelSteps.y * axisMap.forward.y + mmAngVelSteps.z * axisMap.forward.z;
//     // Left wing tip moves forward and right wing tip moves backwards
//     int32_t yawRight = mmAngVelSteps.x * axisMap.up.x + mmAngVelSteps.y * axisMap.up.y + mmAngVelSteps.z * axisMap.up.z;
//     // Nose moves up and tail moves down
//     int32_t pitchUp = mmAngVelSteps.x * axisMap.right.x + mmAngVelSteps.y * axisMap.right.y + mmAngVelSteps.z * axisMap.right.z;
// }