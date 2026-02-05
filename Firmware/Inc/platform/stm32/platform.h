#ifndef PLATFORM_STM32_H
#define PLATFORM_STM32_H

#include "stm32h7xx.h"
#include "stm32h7xx_hal.h"

#include "stm32h7xx_ll_bus.h"
#include "stm32h7xx_ll_dma.h"
#include "stm32h7xx_ll_gpio.h"
#include "stm32h7xx_ll_rcc.h"
#include "stm32h7xx_ll_spi.h"
#include "stm32h7xx_ll_system.h"
#include "stm32h7xx_ll_tim.h"

#define PLAT_MAX_CORES             2U
#define PLAT_CORE_ID_MAKE(NAME)    NAME##_CPUID

#define PLAT_SPI_MAX_DEVS          5U
#define PLAT_SPI_MAX_PIN_SEL       1U

#define PLAT_I2C_MAX_DEVS          2U

#define PLAT_UART_MAX_DEVS         5U
#define PLAT_UART_MAX_PIN_SEL      1U

#define PLAT_DMA_MAX_DEVS          8U

#define PLAT_GPIO_PORT_START_ADDR  GPIOA_BASE
#define PLAT_GPIO_MAX_PORTS        11U
#define PLAT_GPIO_PORT_BYTE_OFFSET 0x400UL
#define PLAT_GPIO_MAX_PINS         16U
#define PLAT_GPIO_PIN_BITS         4U

// clang-format off

#define PLAT_GPIO_CFG_MAKE(MODE, PULL, SPEED) ((GPIOCfg_t){ .mode = (MODE), .pull = (PULL), .speed = (SPEED) })
#define PLAT_GPIO_CFG_AF_PP_NOPULL            PLAT_GPIO_CFG_MAKE (GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH)
#define PLAT_GPIO_CFG_AF_PP_PULLUP            PLAT_GPIO_CFG_MAKE (GPIO_MODE_AF_PP, GPIO_PULLUP, GPIO_SPEED_FREQ_HIGH)
#define PLAT_GPIO_CFG_AF_PP_PULLDOWN          PLAT_GPIO_CFG_MAKE (GPIO_MODE_AF_PP, GPIO_PULLDOWN, GPIO_SPEED_FREQ_HIGH)
#define PLAT_GPIO_CFG_AF_PP_NOPULL            PLAT_GPIO_CFG_MAKE (GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH)
#define PLAT_GPIO_CFG_OUT_PP_NOPULL           PLAT_GPIO_CFG_MAKE (GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH)
#define PLAT_GPIO_CFG_TIM_PWM                 PLAT_GPIO_CFG_MAKE (GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH)
#define PLAT_GPIO_CFG_UART                    PLAT_GPIO_CFG_MAKE (GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_VERY_HIGH)
#define PLAT_GPIO_CFG_GET_MODE(GPIO_CFG)      ((GPIO_CFG).mode)
#define PLAT_GPIO_CFG_GET_PULL(GPIO_CFG)      ((GPIO_CFG).pull)
#define PLAT_GPIO_CFG_GET_SPEED(GPIO_CFG)     ((GPIO_CFG).speed)

// clang-format on
#endif // PLATFORM_STM32_H