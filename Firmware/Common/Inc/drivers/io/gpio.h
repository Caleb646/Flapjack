#ifndef DRIVERS_IO_GPIO_H
#define DRIVERS_IO_GPIO_H

#include "hal.h"
#include "target.h"

#include "core/core.h"

#define GPIO_ENABLE_CLOCK(gpio_port)       \
    do {                                   \
        if ((gpio_port) == GPIOA) {        \
            __HAL_RCC_GPIOA_CLK_ENABLE (); \
        } else if ((gpio_port) == GPIOB) { \
            __HAL_RCC_GPIOB_CLK_ENABLE (); \
        } else if ((gpio_port) == GPIOC) { \
            __HAL_RCC_GPIOC_CLK_ENABLE (); \
        } else if ((gpio_port) == GPIOD) { \
            __HAL_RCC_GPIOD_CLK_ENABLE (); \
        } else if ((gpio_port) == GPIOE) { \
            __HAL_RCC_GPIOE_CLK_ENABLE (); \
        } else if ((gpio_port) == GPIOF) { \
            __HAL_RCC_GPIOF_CLK_ENABLE (); \
        } else if ((gpio_port) == GPIOG) { \
            __HAL_RCC_GPIOG_CLK_ENABLE (); \
        } else if ((gpio_port) == GPIOH) { \
            __HAL_RCC_GPIOH_CLK_ENABLE (); \
        } else if ((gpio_port) == GPIOI) { \
            __HAL_RCC_GPIOI_CLK_ENABLE (); \
        } else if ((gpio_port) == GPIOJ) { \
            __HAL_RCC_GPIOJ_CLK_ENABLE (); \
        } else if ((gpio_port) == GPIOK) { \
            __HAL_RCC_GPIOK_CLK_ENABLE (); \
        }                                  \
    } while (0)


static inline void GPIO_SetHigh (GPIO_TypeDef* pPort, uint16_t pin) {
    pPort->BSRR = (uint32_t)pin;
}

static inline void GPIO_SetLow (GPIO_TypeDef* pPort, uint16_t pin) {
    pPort->BSRR = (uint32_t)pin << 16U;
}

typedef struct {
    GPIO_TypeDef* pPort;
    uint16_t pin;
    eDEVICE_ID_t ownerId;
} IO_t;

#endif /* DRIVERS_IO_GPIO_H */