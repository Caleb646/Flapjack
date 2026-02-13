#ifndef DRIVERS_TIMERS_H
#define DRIVERS_TIMERS_H

#include "hal.h"
#include "target.h"

#define TIMER_ENABLE_CLOCK(tim_instance)      \
    do {                                      \
        if ((tim_instance) == TIM1) {         \
            __HAL_RCC_TIM1_CLK_ENABLE ();     \
        } else if ((tim_instance) == TIM2) {  \
            __HAL_RCC_TIM2_CLK_ENABLE ();     \
        } else if ((tim_instance) == TIM3) {  \
            __HAL_RCC_TIM3_CLK_ENABLE ();     \
        } else if ((tim_instance) == TIM4) {  \
            __HAL_RCC_TIM4_CLK_ENABLE ();     \
        } else if ((tim_instance) == TIM5) {  \
            __HAL_RCC_TIM5_CLK_ENABLE ();     \
        } else if ((tim_instance) == TIM6) {  \
            __HAL_RCC_TIM6_CLK_ENABLE ();     \
        } else if ((tim_instance) == TIM7) {  \
            __HAL_RCC_TIM7_CLK_ENABLE ();     \
        } else if ((tim_instance) == TIM8) {  \
            __HAL_RCC_TIM8_CLK_ENABLE ();     \
        } else if ((tim_instance) == TIM12) { \
            __HAL_RCC_TIM12_CLK_ENABLE ();    \
        } else if ((tim_instance) == TIM13) { \
            __HAL_RCC_TIM13_CLK_ENABLE ();    \
        } else if ((tim_instance) == TIM14) { \
            __HAL_RCC_TIM14_CLK_ENABLE ();    \
        } else if ((tim_instance) == TIM15) { \
            __HAL_RCC_TIM15_CLK_ENABLE ();    \
        } else if ((tim_instance) == TIM16) { \
            __HAL_RCC_TIM16_CLK_ENABLE ();    \
        } else if ((tim_instance) == TIM17) { \
            __HAL_RCC_TIM17_CLK_ENABLE ();    \
        }                                     \
    } while (0)


#endif /* DRIVERS_TIMERS_H */