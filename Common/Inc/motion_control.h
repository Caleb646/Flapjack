#ifndef MOTION_CONTROL_H
#define MOTION_CONTROL_H

#include <stdint.h>
#include "stm32h747xx.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_tim.h"
#include "FreeRTOS.h"
#include "task.h"
#include "common.h"
#include "flight_context.h"

/*
* PID 
*/
typedef struct
{
	uint32_t P, I, D;
} PIDContext;

Vec3 MotionControlPIDUpdateVel(PIDContext pidContext, Vec3 curVel, Vec3 targetVel);

/*
* PWM & Motion Control
*/

typedef struct {
    TIM_HandleTypeDef *pTimerHandle;
    TIM_TypeDef *pTimerRegisters;
    uint32_t timerChannelID;
    uint32_t msMaxDutyCycle;
    uint32_t msMaxPWMCycle;
} MotionControlPWMDescriptor;

typedef struct {
    Vec3 axis;
} MotionControlDescriptor;

typedef struct {
    int32_t minAngle;
    int32_t maxAngle;
    int32_t curAngle;
    MotionControlPWMDescriptor pwmDescriptor;
} MotionControlServo;

typedef struct {
    int32_t curRPM;
    MotionControlPWMDescriptor pwmDescriptor;
} MotionControlMotor;

typedef struct {
    MotionControlServo servo;
    MotionControlMotor motor;
} MotionControlTandemDevice;

typedef struct {
    Vec3 forward;
    Vec3 up;
    Vec3 right;
} AxisMap;

// void MotionControlUpdatePWM(
//     AxisMap axisConf, Vec3 mmVelSteps, Vec3 mmAngVelSteps, void *devs, uint32_t nDevs
// );

#endif // MOTION_CONTROL_H
