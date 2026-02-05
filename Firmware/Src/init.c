#include <stdbool.h>
#include <stdint.h>

#include "common.h"

#include "aero/flight.h"
#include "aero/mixer.h"
#include "aero/pid.h"

#include "drivers/io/gpio.h"

#include "drivers/core/log.h"
#include "drivers/core/sys.h"

#include "drivers/motor.h"

#include "drivers/sensors/sensor.h"

#include "platform/platform.h"

#include "targets/target.h"

eSTATUS_t Aux_Init (void) {

    eSTATUS_t status = eSTATUS_OK;
    if (TARG_MAG_ENABLED ()) {
        status = Mag_Init ();
        RETURN_IF (FJ_FAIL (status), status, "Mag init failed");
    }

    if (TARG_BARO_ENABLED ()) {
        status = Baro_Init ();
        RETURN_IF (FJ_FAIL (status), status, "Baro init failed");
    }

    if (TARG_GPS_ENABLED ()) {
        status = Gps_Init ();
        RETURN_IF (FJ_FAIL (status), status, "GPS init failed");
    }
    // TODO
    if (TARG_FLASH_ENABLED ()) {
        // status = Plat_Flash_Init();
        // RETURN_IF (FJ_FAIL (status), status, "Flash init failed");
    }
    return status;
}


eSTATUS_t Init (void) {

    eSTATUS_t status = eSTATUS_OK;
    if (CORE_IS_PRIMARY ()) {

        status = System_InitPrimaryCore ();
        if (FJ_FAIL (status)) {
            return status;
        }
        status = Acc_Init ();
        RETURN_IF (FJ_FAIL (status), status, "Acc init failed");

        status = Gyro_Init ();
        RETURN_IF (FJ_FAIL (status), status, "Gyro init failed");

        if (!TARG_DUAL_CORE_ENABLED ()) {

            status = Aux_Init ();
            RETURN_IF (FJ_FAIL (status), status, "Aux init failed");
        }

        status = Motors_Init ();
        RETURN_IF (FJ_FAIL (status), status, "Motors init failed");

        status = Servos_Init ();
        RETURN_IF (FJ_FAIL (status), status, "Servos init failed");

        status = Mixer_Init ();
        RETURN_IF (FJ_FAIL (status), status, "Mixer init failed");

        status = PidData_Init ();
        RETURN_IF (FJ_FAIL (status), status, "PID init failed");

        status = FlightData_Init (true);
        RETURN_IF (FJ_FAIL (status), status, "Flight init failed");

    } else {
        status = System_InitSecondaryCore ();
        if (FJ_FAIL (status)) {
            return status;
        }

        status = Aux_Init ();
        RETURN_IF (FJ_FAIL (status), status, "Aux init failed");
    }

    // TODO:
    // 1. init system timer
    // 2. init RX
    // 3. init debug / flash / logging

    return status;
}
