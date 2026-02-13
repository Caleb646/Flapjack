#ifndef MC_SERVOS_H
#define MC_SERVOS_H

#include <stdbool.h>
#include <stdint.h>

#include "drivers/io/gpio.h"

#include "core/core.h"

#include "target.h"

#define SERVO_LEFT_US_DC   500U
#define SERVO_CENTER_US_DC 1500U
#define SERVO_RIGHT_US_DC  2500U

typedef struct {
    TIM_TypeDef* pTimerInstance;
    uint32_t timerAf;
    uint32_t timerChannel;
    GPIO_TypeDef* pPort;
    uint16_t pin;
} ServoHardware_t;

typedef struct {
    TIM_HandleTypeDef* pTimerHandles[BRD_SERVO_COUNT];
    ServoHardware_t* pHardware[BRD_SERVO_COUNT];
} Servos_t;

FJ_DECLARE_SHARED (ServoHardware_t, g_ServosHardware[BRD_SERVO_COUNT]);
FJ_DECLARE_SHARED (TIM_HandleTypeDef, g_ServoTimerHandles[2U]);
FJ_DECLARE_SHARED (Servos_t, g_Servos);

eSTATUS_t Servos_Init (void);
eSTATUS_t Servos_Write (uint16_t const servoVals[BRD_SERVO_COUNT]);

#endif // MC_SERVOS_H