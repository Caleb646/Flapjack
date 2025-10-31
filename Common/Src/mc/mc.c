
#include "mc/mc.h"
#include "fcstate.h"
#include "log/logger.h"
#include <stdbool.h>
#include <stdint.h>

eSTATUS_t MCInitAll (void) {

    eSTATUS_t status = eSTATUS_SUCCESS;

    PID_INIT (&status);
    RETURN_IF (STATUS_FAIL (status), status, "Failed to init PID");

    FILTER_INIT (&status);
    RETURN_IF (STATUS_FAIL (status), status, "Failed to init filter");

    // Let motors and servos be initialized by device module

    return status;
}

eSTATUS_t MCStartAll (void) {

    eSTATUS_t status = eSTATUS_SUCCESS;

    status = PIDStart (PIDGetMutableActivePID ());
    RETURN_IF (STATUS_FAIL (status), status, "Failed to start PID");

    Vec3f startingAttitude = { 0.0F };
    status                 = FilterStart (FilterGetMutableActiveFilter (), 500U, &startingAttitude);
    RETURN_IF (STATUS_FAIL (status), status, "Failed to start filter");

    if (FCStateSetCurrentAttitude (FCStateGetMutableActiveState (), startingAttitude) == false) {
        LOG_ERROR ("Failed to set starting attitude in FC state");
        return eSTATUS_FAILURE;
    }

    status = ActuatorsStart ();
    RETURN_IF (STATUS_FAIL (status), status, "Failed to start actuators");

    return status;
}