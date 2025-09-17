#ifndef PERIPHS_GPIO_H
#define PERIPHS_GPIO_H

#include "common.h"
#include "hal.h"

#define IMU_INT_EXTI_IRQn EXTI9_5_IRQn

eSTATUS_t GPIOInit (void);


#endif // PERIPHS_GPIO_H