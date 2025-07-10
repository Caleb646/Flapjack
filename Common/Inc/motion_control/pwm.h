#ifndef __MOTION_CONTROL_PWM_H
#define __MOTION_CONTROL_PWM_H

#include "common.h"
#include "hal.h"
#include "log.h"

#define PWM_CHECK_OK(pPwmHandle) \
    ((pPwmHandle) != NULL && (pPwmHandle)->pTimerRegisters != NULL)

#define PWM_SET_PRESCALER(pPwmHandle, valPrescaler)          \
    do {                                                     \
        (pPwmHandle)->pTimerRegisters->PSC = (valPrescaler); \
    } while (0)

#define PWM_SET_PERIOD(pPwmHandle, valPeriod)             \
    do {                                                  \
        (pPwmHandle)->pTimerRegisters->ARR = (valPeriod); \
    } while (0)

#define PWM_HZ2US(hz) \
    ((float)(1000000.0F /* <-- prescaled clock, 1 MHz */ / (float)(hz))) // Convert Hz to microseconds

#define PWM_US2HZ(us) \
    ((float)(1000000.0F /* <-- 1 second in us */ / (float)(us))) // Convert microseconds to Hz

#define PWM_US2DC(us, usPeriod) \
    (((float)(us) * (100.0F / (float)(usPeriod)))) // Convert microseconds to duty cycle percentage

#define PWM_SET_COMPARE(pPwmHandle, compare)                                           \
    do {                                                                               \
        if ((pPwmHandle)->timerChannelID == TIM_CHANNEL_1) {                           \
            (pPwmHandle)->pTimerRegisters->CCR1 = (compare);                           \
        } else if ((pPwmHandle)->timerChannelID == TIM_CHANNEL_2) {                    \
            (pPwmHandle)->pTimerRegisters->CCR2 = (compare);                           \
        } else if ((pPwmHandle)->timerChannelID == TIM_CHANNEL_3) {                    \
            (pPwmHandle)->pTimerRegisters->CCR3 = (compare);                           \
        } else if ((pPwmHandle)->timerChannelID == TIM_CHANNEL_4) {                    \
            (pPwmHandle)->pTimerRegisters->CCR4 = (compare);                           \
        } else if ((pPwmHandle)->timerChannelID == TIM_CHANNEL_5) {                    \
            (pPwmHandle)->pTimerRegisters->CCR5 = (compare);                           \
        } else if ((pPwmHandle)->timerChannelID == TIM_CHANNEL_6) {                    \
            (pPwmHandle)->pTimerRegisters->CCR6 = (compare);                           \
        } else {                                                                       \
            LOG_ERROR (                                                                \
            "Invalid timer channel ID: 0x%X", (uint16_t)(pPwmHandle)->timerChannelID); \
        }                                                                              \
    } while (0)

typedef struct {
    // TIM_HandleTypeDef* pTimerHandle;
    TIM_TypeDef* pTimerRegisters;
    uint32_t timerChannelID;
    uint32_t hzPeriod; // PWM period frequency in Hz
} PWMHandle;

STATUS_TYPE PWMInit (PWMHandle* pHandle);
STATUS_TYPE PWMStart (PWMHandle* pHandle);
STATUS_TYPE PWMSend (PWMHandle* pHandle, uint32_t usUpTime);

#endif // __MOTION_CONTROL_PWM_H