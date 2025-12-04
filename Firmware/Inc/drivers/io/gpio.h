#ifndef DRIVERS_GPIO_H
#define DRIVERS_GPIO_H

#include <stdbool.h>
#include <stdint.h>

#include "common.h"

#include "drivers/driver.h"
#include "drivers/io/gpio_defs.h"

#include "platform/platform.h"


FJ_STATIC_ASSERT (PLAT_GPIO_MAX_PORTS < 16, "GPIO id will overflow if more than 15 ports");

DRIVER_DECLARE_ARRAY (GPIO_t, GPIOs, PLAT_GPIO_MAX_PORTS* PLAT_GPIO_MAX_PINS);

GPIO_t* Plat_GPIO_Init (eGPIO_ID_t gpioId, uint8_t ownerId, GPIOCfg_t cfg, uint32_t af);
GPIO_t* Plat_GPIO_GetIO (eGPIO_ID_t gpioId);
GPIO_TypeDef* Plat_GPIO_GetPort (GPIO_t* pIO);
uint16_t Plat_GPIO_GetPin (GPIO_t* pIO);
void Plat_GPIO_Write (GPIO_t* pIO, eGPIO_STATE_t state);
void Plat_GPIO_SetHigh (GPIO_t* pIO);
void Plat_GPIO_SetLow (GPIO_t* pIO);

static inline GPIO_t* GPIO_Init (eGPIO_ID_t gpioId, uint8_t ownerId, GPIOCfg_t cfg, uint32_t af) {
    return Plat_GPIO_Init (gpioId, ownerId, cfg, af);
}

static inline GPIO_t* GPIO_GetIO (eGPIO_ID_t gpioId) {
    return Plat_GPIO_GetIO (gpioId);
}

static inline void GPIO_Write (GPIO_t* pIO, eGPIO_STATE_t state) { // ? pin : (pin << 16U);
    Plat_GPIO_Write (pIO, state);
}

static inline void GPIO_SetHigh (GPIO_t* pIO) {
    Plat_GPIO_SetHigh (pIO);
}

static inline void GPIO_SetLow (GPIO_t* pIO) {
    Plat_GPIO_SetLow (pIO);
}


#endif // DRIVERS_GPIO_H