#include <stdint.h>

#include "common.h"

#include "platform/platform.h"

#include "drivers/io/gpio.h"
#include "drivers/io/gpio_defs.h"

FJ_STATIC_ASSERT (PLAT_GPIO_MAX_PORTS < 16, "GPIO id will overflow if more than 15 ports");
FJ_DEFINE_SHARED (GPIO_t, e_Gpios[PLAT_GPIO_MAX_PORTS * PLAT_GPIO_MAX_PINS]);

eSTATUS_t GPIO_SystemInit (void) {
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

GPIO_t* GPIO_GetIO (eGPIO_ID_t gpioId) {
    return &e_Gpios[GPIO_GetIndex (gpioId)];
}

uint32_t GPIO_GetIndex (eGPIO_ID_t gpioId) {
    return (GPIO_ID_TO_PORT_INDEX (gpioId) * PLAT_GPIO_MAX_PINS) + (GPIO_ID_TO_PIN_INDEX (gpioId));
}