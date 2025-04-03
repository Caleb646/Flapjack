#ifndef MOTION_CONTROL_ACTUATORS_H
#define MOTION_CONTROL_ACTUATORS_H

#include <stdint.h>
#include "stm32h747xx.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_tim.h"
#include "FreeRTOS.h"
#include "task.h"
#include "common.h"
// #include "flight_context.h"

int8_t TargetAttitudeThrottleUpdate(void);

/*
* PID 
*/
typedef struct
{
	float P, I, D;
    Vec3f attitudeError;
} PIDContext;

int8_t PIDUpdateAttitude(
    PIDContext *pidContext, 
    Vec3f currentAttitude, 
    Vec3f targetAttitude, 
    float dt, 
    Vec3f *pOutputAttitude
);


/*
* PWM & Motion Control
*/
typedef struct {
    TIM_HandleTypeDef *pTimerHandle;
    TIM_TypeDef *pTimerRegister;
    uint32_t timerChannelID;
} PWMHandle;

typedef struct {
    uint32_t usMinDutyCycle;
    uint32_t usMaxDutyCycle;
    uint32_t usMaxPWMCycle;
    // Between 0 and 1
    float scaledDutyCycle;
} PWMDescriptor;

typedef struct {
    int32_t minAngle;
    int32_t maxAngle;
    int32_t curAngle;
    PWMHandle pwmInterface;
    PWMDescriptor pwmDescriptor;
} Servo;

typedef struct {
    PWMHandle pwmInterface;
    PWMDescriptor pwmDescriptor;
} Motor;

typedef struct {
    Vec3 forward;
    Vec3 up;
    Vec3 right;
} AxisMap;

int8_t PID2PWMMixer(Vec3f pidAttitude, float targetThrottle);
int8_t MotionControlInit(PWMHandle leftMotorInter, PWMHandle leftServoInter);

// void MotionControlUpdatePWM(
//     AxisMap axisConf, Vec3 mmVelSteps, Vec3 mmAngVelSteps, void *devs, uint32_t nDevs
// );

#endif // MOTION_CONTROL_ACTUATORS_H
