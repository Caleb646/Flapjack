#include <stdint.h>

#include "platform/stm32/platform.h"

#include "drivers/io/gpio.h"
#include "drivers/io/gpio_defs.h"


eSTATUS_t Plat_GPIO_SystemInit (void) {

    // NOTE: Right now a GPIO_t is just an index which is used with
    // the port offset to get the actual port

    // for (uint32_t portId = 0; portId < PLAT_GPIO_MAX_PORTS; ++portId) {
    //     for (uint32_t pinId = 0; pinId < PLAT_GPIO_MAX_PINS; ++pinId) {
    //         GPIO_t* pIO = &GPIOs[(portId * PLAT_GPIO_MAX_PINS) + pinId];
    //         pIO->pPort  = (GPIO_TypeDef*)((uintptr_t)GPIOA + (portId * PLAT_GPIO_PORT_BYTE_OFFSET));
    //         pIO->pin    = GPIO_PIN_0 << pinId;
    //         pIO->ownerId = eDEVICE_ID_NULL;
    //     }
    // }

    return eSTATUS_OK;
}

GPIO_t* Plat_GPIO_Init (eGPIO_ID_t gpioId, uint8_t ownerId, GPIOCfg_t cfg, uint32_t af) {
}

GPIO_t* Plat_GPIO_GetIO (eGPIO_ID_t gpioId) {
    return GPIOs_GetMutable (PLAT_GPIO_ID_TO_INDEX (gpioId));
}

GPIO_TypeDef* Plat_GPIO_GetPort (GPIO_t* pIO) {

    if (!pIO) {
        return NULL;
    }
    return (GPIO_TypeDef*)(PLAT_GPIO_PORT_START_ADDR +
                           (PLAT_GPIO_PORT_BYTE_OFFSET * (PLAT_GPIO_ID_TO_PORT_INDEX (pIO->id))));
}

uint16_t Plat_GPIO_GetPin (GPIO_t* pIO) {

    if (!pIO) {
        return eGPIO_PINID_NULL;
    }
    return (uint16_t)(1U << PLAT_GPIO_ID_TO_PIN_INDEX (pIO->id));
}

void Plat_GPIO_Write (GPIO_t* pIO, eGPIO_STATE_t state) {

    if (!pIO) {
        return;
    }
    Plat_GPIO_GetPort (pIO)->BSRR = (uint32_t)Plat_GPIO_GetPin (pIO) << ((state != eGPIO_STATE_HIGH) * 16U);
}

void Plat_GPIO_SetHigh (GPIO_t* pIO) {

    if (!pIO) {
        return;
    }
    Plat_GPIO_GetPort (pIO)->BSRR = Plat_GPIO_GetPin (pIO);
}

void Plat_GPIO_SetLow (GPIO_t* pIO) {

    if (!pIO) {
        return;
    }
    Plat_GPIO_GetPort (pIO)->BSRR = ((uint32_t)Plat_GPIO_GetPin (pIO) << 16U);
}