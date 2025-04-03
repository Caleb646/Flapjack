#include <string.h>
#include "motion_control/actuators.h"

int8_t PIDUpdateAttitude(
    PIDContext *pidContext, 
    Vec3f currentAttitude, 
    Vec3f targetAttitude, 
    float dt, 
    Vec3f *pOutputAttitude
)
{
    
}

/*
* PWM & Motion Control
*/
static Motor leftMotor;
static Servo leftServo;
// static Motor rightMotor;
// static Servo rightServo;
int8_t PID2PWMMixer(Vec3f pidAttitude, float targetThrottle)
{
    leftMotor.pwmDescriptor.scaledDutyCycle;

    return 0;
}

int8_t MotionControlInit(
    PWMInterfaceDescriptor leftMotorInter, PWMInterfaceDescriptor leftServoInter
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

    return 0;
}

void MotionControlUpdatePWM(
    AxisMap axisMap, Vec3 velMax, Vec3 velAngMax, Vec3 mmVelSteps, Vec3 mmAngVelSteps, void *devs, uint32_t nDevs
)
{
    int32_t forward = mmVelSteps.x * axisMap.forward.x + mmVelSteps.y * axisMap.forward.y + mmVelSteps.z * axisMap.forward.z;
    int32_t up = mmVelSteps.x * axisMap.up.x + mmVelSteps.y * axisMap.up.y + mmVelSteps.z * axisMap.up.z;
    int32_t right = mmVelSteps.x * axisMap.right.x + mmVelSteps.y * axisMap.right.y + mmVelSteps.z * axisMap.right.z;

    // Left wing tip is higher than right wing tip
    int32_t rollRight = mmAngVelSteps.x * axisMap.forward.x + mmAngVelSteps.y * axisMap.forward.y + mmAngVelSteps.z * axisMap.forward.z;
    // Left wing tip moves forward and right wing tip moves backwards
    int32_t yawRight = mmAngVelSteps.x * axisMap.up.x + mmAngVelSteps.y * axisMap.up.y + mmAngVelSteps.z * axisMap.up.z;
    // Nose moves up and tail moves down
    int32_t pitchUp = mmAngVelSteps.x * axisMap.right.x + mmAngVelSteps.y * axisMap.right.y + mmAngVelSteps.z * axisMap.right.z;
}