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

Vec3 PIDCalcNewVelocitySteps(PIDContext pidContext, Vec3 curVel, Vec3 targetVel);

/*
* PWM & Motion Control
*/
typedef struct {
    TIM_HandleTypeDef *pTimerHandle;
    TIM_TypeDef *pTimerRegisters;
    uint32_t timerChannelID;
    uint32_t msMaxDutyCycle;
    uint32_t msMaxPWMCycle;
} MotionControlDevice;

void PWMMotionControlDevicesUpdate(
    FlightContext *pFlightContext, 
    MotionControlDevice *pDevices, 
    uint32_t nDevices
);

#endif // MOTION_CONTROL_H
