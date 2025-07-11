#ifndef __MOTION_CONTROL_PWM_H
#define __MOTION_CONTROL_PWM_H

#include "common.h"
#include "hal.h"
#include "log.h"

#define PWM_CHECK_OK(pPwmHandle) \
    ((pPwmHandle) != NULL && (pPwmHandle)->timer.Instance != NULL)

#define PWM_HZ2US(hz) \
    ((float)(1000000.0F /* <-- prescaled clock, 1 MHz */ / (float)(hz))) // Convert Hz to microseconds

#define PWM_US2HZ(us) \
    ((float)(1000000.0F /* <-- 1 second in us */ / (float)(us))) // Convert microseconds to Hz

#define PWM_US2DC(us, usPeriod) \
    (((float)(us) * (100.0F / (float)(usPeriod)))) // Convert microseconds to duty cycle percentage

#define PWM_SET_PRESCALER(pPwmHandle, valPrescaler)         \
    do {                                                    \
        (pPwmHandle)->timer.Instance->PSC = (valPrescaler); \
    } while (0)

#define PWM_SET_PERIOD(pPwmHandle, valPeriod)            \
    do {                                                 \
        (pPwmHandle)->timer.Instance->ARR = (valPeriod); \
    } while (0)

#define PWM_SET_COMPARE(pPwmHandle, compare)                                      \
    do {                                                                          \
        if ((pPwmHandle)->channelID == TIM_CHANNEL_1) {                           \
            (pPwmHandle)->timer.Instance->CCR1 = (compare);                       \
        } else if ((pPwmHandle)->channelID == TIM_CHANNEL_2) {                    \
            (pPwmHandle)->timer.Instance->CCR2 = (compare);                       \
        } else if ((pPwmHandle)->channelID == TIM_CHANNEL_3) {                    \
            (pPwmHandle)->timer.Instance->CCR3 = (compare);                       \
        } else if ((pPwmHandle)->channelID == TIM_CHANNEL_4) {                    \
            (pPwmHandle)->timer.Instance->CCR4 = (compare);                       \
        } else if ((pPwmHandle)->channelID == TIM_CHANNEL_5) {                    \
            (pPwmHandle)->timer.Instance->CCR5 = (compare);                       \
        } else if ((pPwmHandle)->channelID == TIM_CHANNEL_6) {                    \
            (pPwmHandle)->timer.Instance->CCR6 = (compare);                       \
        } else {                                                                  \
            LOG_ERROR (                                                           \
            "Invalid timer channel ID: 0x%X", (uint16_t)(pPwmHandle)->channelID); \
        }                                                                         \
    } while (0)

typedef struct {
    TIM_TypeDef* pTimer;
    uint32_t channelID;
    uint32_t hzPeriod; // PWM period frequency in Hz
} PWMConfig;

typedef struct {
    TIM_HandleTypeDef timer;
    uint32_t channelID;
} PWMHandle;


typedef struct {
    TIM_TypeDef* pTimer;
    uint32_t channelID;
    uint32_t hzPeriod; // PWM period frequency in Hz
} PWMDMAConfig;

typedef struct {
    PWMHandle handle;
    DMA_HandleTypeDef dma;
} PWMDMAHandle;

STATUS_TYPE PWMInit (PWMConfig* pConfig, PWMHandle* pOutHandle);
STATUS_TYPE PWMDMAInit (PWMDMAHandle* pOutHandle, PWMConfig* pConfig);
STATUS_TYPE PWMStart (PWMHandle* pHandle);
STATUS_TYPE PWMSend (PWMHandle* pHandle, uint32_t usUpTime);

#endif // __MOTION_CONTROL_PWM_H