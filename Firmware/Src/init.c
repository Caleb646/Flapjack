#include <stdbool.h>
#include <stdint.h>

#include "common.h"

#include "core/core.h"

#include "aero/flight.h"
#include "aero/mixer.h"
#include "aero/pid.h"

#include "drivers/io/gpio.h"

#include "drivers/sensors/sensor.h"

eSTATUS_t Init (void) {

    eSTATUS_t status = GPIO_SystemInit ();
    RETURN_IF (FJ_FAIL (status), status, "GPIO system init failed");

    status = Sensors_Init ();
    RETURN_IF (FJ_FAIL (status), status, "Sensors init failed");

    status = Mixer_Init ();
    RETURN_IF (FJ_FAIL (status), status, "Mixer init failed");

    status = Pid_Init ();
    RETURN_IF (FJ_FAIL (status), status, "PID init failed");

    status = Flight_Init (true);
    RETURN_IF (FJ_FAIL (status), status, "Flight init failed");

    return eSTATUS_SUCCESS;
}
