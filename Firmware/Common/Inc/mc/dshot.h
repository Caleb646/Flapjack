#ifndef __MOTION_CONTROL_DSHOT_H__
#define __MOTION_CONTROL_DSHOT_H__

#include "hal.h"
#include "target.h"

#include "conf/board.h"
#include "conf/conf.h"
#include "conf/ids.h"

#include "core/core.h"

#include "peripheral/gpio.h"
#include "peripheral/timer.h"

#define DSHOT_DMA_BUFFER_SIZE 18U /* resolution + frame reset (2us) */
#define DSHOT_FRAME_SIZE      16U
#define DSHOT_MIN_THROTTLE    48U
#define DSHOT_MAX_THROTTLE    2047U
#define DSHOT_RANGE           (DSHOT_MAX_THROTTLE - DSHOT_MIN_THROTTLE)

typedef struct {
    uint32_t timerChannel;
    GPIO_TypeDef* pPort;
    uint16_t pin;
} DShotBBMotorPin_t;

typedef struct {
    TIM_TypeDef* pTimerInstance;
    uint32_t timerAf;
    DShotBBMotorPin_t motorPins[BRD_MOTOR_COUNT];
} DShotBBHardware_t;

typedef struct {
    TIM_HandleTypeDef timerHandle;
    DShotBBHardware_t hardware;
    uint32_t ticksFor_0;
    uint32_t ticksFor_1;
} DShotBB_t;

FJ_DECLARE_SHARED (DShotBB_t, g_DShotBB);

eSTATUS_t DShotBB_Init (void);
eSTATUS_t DShotBB_Write (uint16_t motorVals[BRD_MOTOR_COUNT]);

#endif /* __MOTION_CONTROL_DSHOT_H__ */