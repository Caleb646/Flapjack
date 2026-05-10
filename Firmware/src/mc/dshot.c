#include "hal.h"
#include "target.h"

#include "mc/dshot.h"

#include "core/core.h"

#include "drivers/io/gpio.h"
#include "drivers/timer.h"


#include <stdint.h>


FJ_DEFINE_SHARED (DShotBB_t, g_DShotBB) = {
    .timerHandle = { 0 },
    .hardware    = { .pTimerInstance = BRD_GET_TIMER_INSTANCE (MOTOR_1),
                     .timerAf        = BRD_GET_TIMER_AF (MOTOR_1),
                     .motorPins      = { { .timerChannel = BRD_GET_TIMER_CHANNEL (MOTOR_1),
                                           .pPort        = BRD_GET_GPIO_PORT (MOTOR_1),
                                           .pin          = BRD_GET_GPIO_PIN (MOTOR_1) },
#if defined(MOTOR_2_ENABLED) && (MOTOR_2_ENABLED == 1U)
                                 { .timerChannel = BRD_GET_TIMER_CHANNEL (MOTOR_2),
                                   .pPort        = BRD_GET_GPIO_PORT (MOTOR_2),
                                   .pin          = BRD_GET_GPIO_PIN (MOTOR_2) }
#endif
                  } }
};

#ifndef UNIT_TEST
static uint16_t DShotPreparePacket (uint16_t value);
#endif

// eSTATUS_t DShotBB_Init_()

eSTATUS_t DShotBB_Init (void) {

    DShotBB_t* pDShotBB = &g_DShotBB;
    TIMER_ENABLE_CLOCK (pDShotBB->hardware.pTimerInstance);
    TIM_HandleTypeDef* pTimerHandle      = &pDShotBB->timerHandle;
    pTimerHandle->Instance               = pDShotBB->hardware.pTimerInstance;
    pTimerHandle->Init.Prescaler         = 0;
    pTimerHandle->Init.CounterMode       = TIM_COUNTERMODE_UP;
    pTimerHandle->Init.Period            = 0;
    pTimerHandle->Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    pTimerHandle->Init.RepetitionCounter = 0;
    pTimerHandle->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    // deinit first
    if (HAL_TIM_PWM_DeInit (pTimerHandle) != HAL_OK) {
        LOG_ERROR ("Failed to deinit DShot timer");
        return eSTATUS_FAILURE;
    }
    if (HAL_TIM_PWM_Init (pTimerHandle) != HAL_OK) {
        LOG_ERROR ("Failed to initialize DShot timer");
        return eSTATUS_FAILURE;
    }

    __HAL_TIM_SET_AUTORELOAD (pTimerHandle, 427 - 1);
    pDShotBB->ticksFor_1 = 320; // 75% of 6.67 us / 427 ticks
    pDShotBB->ticksFor_0 = 160; // 37.5% of 6.67 us / 427 ticks

    GPIO_InitTypeDef gpioInit = { 0 };
    gpioInit.Mode             = GPIO_MODE_AF_PP;
    gpioInit.Pull             = GPIO_NOPULL;
    gpioInit.Speed            = GPIO_SPEED_FREQ_VERY_HIGH;
    // NOTE: af is 0 if manually setting the pin to high/low without using timer,
    // so set alternate function only when using timer
    // gpioInit.Alternate = 0U;
    gpioInit.Alternate = pDShotBB->hardware.timerAf;
    for (uint8_t i = 0; i < BRD_MOTOR_COUNT; ++i) {
        DShotBBMotorPin_t motorPin = pDShotBB->hardware.motorPins[i];
        if (!motorPin.pPort) {
            continue;
        }
        GPIO_ENABLE_CLOCK (motorPin.pPort);
        gpioInit.Pin = motorPin.pin;
        HAL_GPIO_Init (motorPin.pPort, &gpioInit);
        GPIO_SetLow (motorPin.pPort, motorPin.pin);

        TIM_OC_InitTypeDef sConfig = { 0 };
        sConfig.OCMode             = TIM_OCMODE_PWM1;
        sConfig.OCPolarity         = TIM_OCPOLARITY_HIGH;
        sConfig.Pulse              = 0;
        sConfig.OCNPolarity        = TIM_OCNPOLARITY_HIGH;
        sConfig.OCFastMode         = TIM_OCFAST_DISABLE;
        sConfig.OCIdleState        = TIM_OCIDLESTATE_RESET;
        sConfig.OCNIdleState       = TIM_OCNIDLESTATE_RESET;
        if (HAL_TIM_PWM_ConfigChannel (pTimerHandle, &sConfig, motorPin.timerChannel) != HAL_OK) {
            LOG_ERROR ("Failed to configure PWM channel");
            return eSTATUS_FAILURE;
        }
    }
    return eSTATUS_SUCCESS;
}

eSTATUS_t DShotBB_Write (uint16_t motorVals[BRD_MOTOR_COUNT]) {

    DShotBB_t* pDShotBB                                      = &g_DShotBB;
    TIM_HandleTypeDef* pTimerHandle                          = &pDShotBB->timerHandle;
    uint16_t buffers[BRD_MOTOR_COUNT][DSHOT_DMA_BUFFER_SIZE] = { 0 };
    for (uint32_t i = 0; i < BRD_MOTOR_COUNT; ++i) {
        uint16_t packet = DShotPreparePacket (motorVals[i]);
        for (uint16_t j = 0; j < 16U; ++j) {
            buffers[i][j] = (packet & 0x8000U) ? pDShotBB->ticksFor_1 : pDShotBB->ticksFor_0;
            packet <<= 1U;
        }
        buffers[i][16] = 0;
        buffers[i][17] = 0;
    }

    __HAL_TIM_CLEAR_FLAG (pTimerHandle, TIM_FLAG_UPDATE);
    for (uint32_t i = 0; i < BRD_MOTOR_COUNT; ++i) {
        __HAL_TIM_SET_COMPARE (pTimerHandle, pDShotBB->hardware.motorPins[i].timerChannel, buffers[i][0]);
    }

    __disable_irq ();
    for (uint32_t i = 0; i < BRD_MOTOR_COUNT; ++i) {
        HAL_TIM_PWM_Start (pTimerHandle, pDShotBB->hardware.motorPins[i].timerChannel);
    }

    for (uint32_t bitPos = 1; bitPos < DSHOT_DMA_BUFFER_SIZE; ++bitPos) {

        while (__HAL_TIM_GET_FLAG (pTimerHandle, TIM_FLAG_UPDATE) == RESET) {
        }
        __HAL_TIM_CLEAR_FLAG (pTimerHandle, TIM_FLAG_UPDATE);
        for (uint32_t i = 0; i < BRD_MOTOR_COUNT; ++i) {
            __HAL_TIM_SET_COMPARE (pTimerHandle, pDShotBB->hardware.motorPins[i].timerChannel, buffers[i][bitPos]);
        }
    }

    __enable_irq ();
    for (uint32_t i = 0; i < BRD_MOTOR_COUNT; ++i) {
        HAL_TIM_PWM_Stop (pTimerHandle, pDShotBB->hardware.motorPins[i].timerChannel);
    }
    return eSTATUS_SUCCESS;
}


STATIC uint16_t DShotPreparePacket (uint16_t value) {

    uint16_t packet         = 0U;
    uint8_t dshot_telemetry = false;

    packet = ((uint32_t)value << 1U) | (dshot_telemetry ? 1U : 0U);

    // compute checksum
    uint16_t csum      = 0;
    uint16_t csum_data = packet;

    for (uint16_t i = 0; i < 3U; i++) {
        csum ^= csum_data; // xor data by nibbles
        csum_data >>= 4U;
    }

    csum &= 0xFU;
    packet = ((uint32_t)packet << 4U) | csum;

    return packet;
}