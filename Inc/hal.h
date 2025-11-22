#ifndef HAL_H
#define HAL_H

#ifdef UNIT_TEST

#include "hal_stub.h"
// #include "mock_stm32h7xx_hal_hsem.h"
// // #include "mock_stm32h7xx_hal_rcc.h"
// #include "mock_stm32h7xx_hal_gpio.h"
// // #include "mock_stm32h7xx_hal_dma.h"
// // #include "mock_stm32h7xx_hal_tim.h"
// #include "mock_stm32h7xx_hal_i2c.h"
// #include "mock_stm32h7xx_hal_spi.h"
// #include "mock_stm32h7xx_hal_uart.h"

#include "stm32h7xx_hal_hsem.h"
// #include "mock_stm32h7xx_hal_rcc.h"
#include "stm32h7xx_hal_gpio.h"
// #include "mock_stm32h7xx_hal_dma.h"
// #include "mock_stm32h7xx_hal_tim.h"
#include "stm32h7xx_hal_dma.h"
#include "stm32h7xx_hal_dma_ex.h"
#include "stm32h7xx_hal_i2c.h"
#include "stm32h7xx_hal_spi.h"
#include "stm32h7xx_hal_uart.h"

#else

#include "cmsis_os.h"
#include "stm32h7xx.h"
#include "stm32h7xx_hal.h"

#endif

#endif // HAL_H