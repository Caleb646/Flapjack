

#include "target.h"

#include "devices/servos.h"

#include "core/core.h"

#include "drivers/io/gpio.h"
#include "drivers/timer.h"


FJ_DEFINE_SHARED (ServoHardware_t, g_ServosHardware[BRD_SERVO_COUNT]) = {
    { .pTimerInstance = BRD_GET_TIMER_INSTANCE (SERVO_1),
      .timerAf        = BRD_GET_TIMER_AF (SERVO_1),
      .timerChannel   = BRD_GET_TIMER_CHANNEL (SERVO_1),
      .pPort          = BRD_GET_GPIO_PORT (SERVO_1),
      .pin            = BRD_GET_GPIO_PIN (SERVO_1) },
};

FJ_DEFINE_SHARED (TIM_HandleTypeDef, g_ServoTimerHandles[2U]) = { 0 };
FJ_DEFINE_SHARED (Servos_t, g_Servos)                         = { 0 };

static TIM_HandleTypeDef* GetTimerHandleByTimerInstanceOrFree (TIM_TypeDef* pTimerInstance) {

    for (uint32_t i = 0; i < BRD_SERVO_COUNT; ++i) {
        if (g_ServosHardware[i].pTimerInstance == pTimerInstance) {
            return &g_ServoTimerHandles[i];
        }
    }

    for (uint32_t i = 0; i < BRD_SERVO_COUNT; ++i) {
        if (g_ServosHardware[i].pTimerInstance == NULL) {
            return &g_ServoTimerHandles[i];
        }
    }
    return NULL;
}

eSTATUS_t Servos_Init (void) {

    Servos_t* pServos = &g_Servos;

    for (uint32_t i = 0; i < BRD_SERVO_COUNT; ++i) {

        pServos->pTimerHandles[i] = GetTimerHandleByTimerInstanceOrFree (g_ServosHardware[i].pTimerInstance);
        pServos->pHardware[i] = &g_ServosHardware[i];

        TIM_HandleTypeDef* pTimerHandle = pServos->pTimerHandles[i];
        ServoHardware_t* pHardware      = pServos->pHardware[i];

        if (!pTimerHandle || !pHardware) {
            LOG_ERROR ("Failed to get timer handle for servo %u", i);
            return eSTATUS_FAILURE;
        }

        TIMER_ENABLE_CLOCK (pHardware->pTimerInstance);
        GPIO_ENABLE_CLOCK (pHardware->pPort);
        GPIO_InitTypeDef gpioInit = { .Pin       = pHardware->pin,
                                      .Mode      = GPIO_MODE_AF_PP,
                                      .Pull      = GPIO_NOPULL,
                                      .Speed     = GPIO_SPEED_FREQ_VERY_HIGH,
                                      .Alternate = pHardware->timerAf };
        HAL_GPIO_Init (pHardware->pPort, &gpioInit);
        GPIO_SetLow (pHardware->pPort, pHardware->pin);

        TIM_OC_InitTypeDef sConfig = { 0 };
        sConfig.OCMode             = TIM_OCMODE_PWM1;
        sConfig.OCPolarity         = TIM_OCPOLARITY_HIGH;
        sConfig.Pulse              = 0U; // default to 0 us pulse width (servo off)
        sConfig.OCNPolarity        = TIM_OCNPOLARITY_HIGH;
        sConfig.OCFastMode         = TIM_OCFAST_DISABLE;
        sConfig.OCIdleState        = TIM_OCIDLESTATE_RESET;
        sConfig.OCNIdleState       = TIM_OCNIDLESTATE_RESET;
        if (HAL_TIM_PWM_ConfigChannel (pTimerHandle, &sConfig, pHardware->timerChannel) != HAL_OK) {
            LOG_ERROR ("Failed to configure PWM channel");
            return eSTATUS_FAILURE;
        }

        // NOTE: If instance is set the timer handle has already been initialized
        if (!pTimerHandle->Instance) {
            continue;
        }

        pTimerHandle->Instance               = pHardware->pTimerInstance;
        pTimerHandle->Init.Prescaler         = 64 - 1;
        pTimerHandle->Init.CounterMode       = TIM_COUNTERMODE_UP;
        pTimerHandle->Init.Period            = 20000 - 1; // 20 ms period or 50 Hz
        pTimerHandle->Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
        pTimerHandle->Init.RepetitionCounter = 0;
        pTimerHandle->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
        // deinit first
        if (HAL_TIM_PWM_DeInit (pTimerHandle) != HAL_OK) {
            LOG_ERROR ("Failed to deinit Servo timer");
            return eSTATUS_FAILURE;
        }
        if (HAL_TIM_PWM_Init (pTimerHandle) != HAL_OK) {
            LOG_ERROR ("Failed to initialize Servo timer");
            return eSTATUS_FAILURE;
        }
    }

    for (uint32_t i = 0; i < BRD_SERVO_COUNT; ++i) {
        if (HAL_TIM_PWM_Start (pServos->pTimerHandles[i], pServos->pHardware[i]->timerChannel) != HAL_OK) {
            LOG_ERROR ("Failed to start PWM for servo %u", i);
            return eSTATUS_FAILURE;
        }
    }

    return eSTATUS_SUCCESS;
}

eSTATUS_t Servos_Write (uint16_t const servoVals[BRD_SERVO_COUNT]) {

    Servos_t* pServos = &g_Servos;
    for (uint32_t i = 0; i < BRD_SERVO_COUNT; ++i) {
        __HAL_TIM_SET_COMPARE (pServos->pTimerHandles[i], pServos->pHardware[i]->timerChannel, servoVals[i]);
    }
    return eSTATUS_SUCCESS;
}
