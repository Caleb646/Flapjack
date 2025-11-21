#ifndef HAL_H
#define HAL_H

#ifdef UNIT_TEST

#include "hal_stub.h"

#else

#include "cmsis_os.h"
#include "stm32h7xx.h"
#include "stm32h7xx_hal.h"

#endif

#endif // HAL_H