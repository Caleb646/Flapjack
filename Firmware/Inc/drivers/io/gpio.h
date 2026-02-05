#ifndef DRIVERS_GPIO_H
#define DRIVERS_GPIO_H

#include <stdbool.h>
#include <stdint.h>

#include "common.h"

#include "drivers/io/gpio_defs.h"

FJ_DECLARE_SHARED (GPIO_t, e_Gpios[]);

GPIO_t* Plat_GPIO_Init (eGPIO_ID_t gpioId, uint8_t ownerId, GPIOCfg_t cfg, uint32_t af);
GPIO_TypeDef* Plat_GPIO_GetPort (GPIO_t* pIO);
uint16_t Plat_GPIO_GetPin (GPIO_t* pIO);
void Plat_GPIO_Write (GPIO_t* pIO, eGPIO_STATE_t state);
void Plat_GPIO_SetHigh (GPIO_t* pIO);
void Plat_GPIO_SetLow (GPIO_t* pIO);

eSTATUS_t GPIO_SystemInit (void);

static inline GPIO_t* GPIO_Init (eGPIO_ID_t gpioId, uint8_t ownerId, GPIOCfg_t cfg, uint32_t af) {
    return Plat_GPIO_Init (gpioId, ownerId, cfg, af);
}

GPIO_t* GPIO_GetIO (eGPIO_ID_t gpioId);

static inline GPIO_TypeDef* GPIO_GetPort (GPIO_t* pIO) {
    return Plat_GPIO_GetPort (pIO);
}

static inline uint16_t GPIO_GetPin (GPIO_t* pIO) {
    return Plat_GPIO_GetPin (pIO);
}

uint32_t GPIO_GetIndex (eGPIO_ID_t gpioId);

static inline uint32_t GPIO_GetPortIndex (eGPIO_ID_t gpioId) {
    return GPIO_ID_TO_PORT_INDEX (gpioId);
}

static inline uint32_t GPIO_GetPinIndex (eGPIO_ID_t gpioId) {
    return GPIO_ID_TO_PIN_INDEX (gpioId);
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