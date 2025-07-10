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
        (pPwmHandle)->prescaler            = (valPrescaler); \
    } while (0)

#define PWM_SET_PERIOD(pPwmHandle, valPeriod)             \
    do {                                                  \
        (pPwmHandle)->pTimerRegisters->ARR = (valPeriod); \
        (pPwmHandle)->period               = (valPeriod); \
    } while (0)

#define PWM_SET_COMPARE(pPwmHandle, compare)            \
    (                                                   \
    ((pPwmHandle)->timerChannelID == TIM_CHANNEL_1) ?   \
    ((pPwmHandle)->pTimerRegisters->CCR1 = (compare)) : \
    ((pPwmHandle)->timerChannelID == TIM_CHANNEL_2) ?   \
    ((pPwmHandle)->pTimerRegisters->CCR2 = (compare)) : \
    ((pPwmHandle)->timerChannelID == TIM_CHANNEL_3) ?   \
    ((pPwmHandle)->pTimerRegisters->CCR3 = (compare)) : \
    ((pPwmHandle)->timerChannelID == TIM_CHANNEL_4) ?   \
    ((pPwmHandle)->pTimerRegisters->CCR4 = (compare)) : \
    ((pPwmHandle)->timerChannelID == TIM_CHANNEL_5) ?   \
    ((pPwmHandle)->pTimerRegisters->CCR5 = (compare)) : \
    ((pPwmHandle)->pTimerRegisters->CCR6 = (compare)))

typedef struct {
    // TIM_HandleTypeDef* pTimerHandle;
    TIM_TypeDef* pTimerRegisters;
    uint32_t timerChannelID;
    uint32_t period;    // ARR Auto-Reload Register
    uint32_t prescaler; // PSC Prescaler
} PWMHandle;

STATUS_TYPE PWMInit (PWMHandle* pHandle);
STATUS_TYPE PWMStart (PWMHandle* pHandle);
STATUS_TYPE PWMSend (PWMHandle* pHandle, uint32_t usTargetDutyCycle);

#endif // __MOTION_CONTROL_PWM_H