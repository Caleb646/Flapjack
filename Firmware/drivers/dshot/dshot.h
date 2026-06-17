#ifndef __MOTION_CONTROL_DSHOT_H__
#define __MOTION_CONTROL_DSHOT_H__

#include "hal.h"
#include "target.h"

#include "core/core.h"

#include "drivers/dma.h"
#include "drivers/io/gpio.h"
#include "drivers/timer.h"


#define DSHOT_DMA_BUFFER_SIZE 18U  /* kept for unit-test compat: 16 data + 2 reset bits */
#define DSHOT_FRAME_SIZE      16U
#define DSHOT_MIN_THROTTLE    48U
#define DSHOT_MAX_THROTTLE    2047U
#define DSHOT_RANGE           (DSHOT_MAX_THROTTLE - DSHOT_MIN_THROTTLE)

/* Legacy timing constants (unit tests validate these ratios) */
#define DSHOT_ARR         426U  /* 427 ticks = 6.67 us/bit at 64 MHz */
#define DSHOT_TICKS_FOR_1 320U  /* 75% high */
#define DSHOT_TICKS_FOR_0 160U  /* 37.5% high */

/* Pacer-timer + GPIO BSRR DMA constants (8 samples/bit, TIM6 ARR=52) */
#define DSHOT_PACER_ARR       52U   /* 53 ticks/sample at 64 MHz → 828 ns */
#define DSHOT_SAMPLES_PER_BIT 8U
#define DSHOT_SAMPLES_FOR_1   6U    /* 6/8 = 75% */
#define DSHOT_SAMPLES_FOR_0   3U    /* 3/8 = 37.5% */
#define DSHOT_BUFFER_SIZE     144U  /* 8 × 18 (16 data + 2 reset) */

typedef struct {
    uint32_t timerChannel;
    GPIO_TypeDef* pPort;
    uint32_t pin;
} DShotBBMotorPin_t;

typedef struct {
    TIM_TypeDef* pTimerInstance;
    uint32_t timerAf;
    DShotBBMotorPin_t motorPins[BRD_MOTOR_COUNT];
} DShotBBHardware_t;

typedef struct {
    TIM_HandleTypeDef tim6Handle;
    DmaHandle_t*      pDmaHandle;
    DShotBBHardware_t hardware;
    uint32_t          buffer[DSHOT_BUFFER_SIZE];
    volatile bool     txDone;
} DShotBB_t;

FJ_DECLARE_SHARED (DShotBB_t, g_DShotBB);

eSTATUS_t DShotBB_Init (void);
eSTATUS_t DShotBB_Write (uint16_t motorVals[BRD_MOTOR_COUNT]);

#ifdef UNIT_TEST
uint16_t DShotPreparePacket (uint16_t value);
#endif

#endif /* __MOTION_CONTROL_DSHOT_H__ */