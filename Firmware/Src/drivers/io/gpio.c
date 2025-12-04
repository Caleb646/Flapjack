#include <stdint.h>

#include "common.h"

#include "platform/platform.h"

#include "drivers/driver.h"

#include "drivers/io/gpio.h"
#include "drivers/io/gpio_defs.h"

DRIVER_DEFINE_ARRAY (GPIO_t, GPIOs, PLAT_GPIO_MAX_PORTS* PLAT_GPIO_MAX_PINS);