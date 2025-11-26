
#include "mc/mc.h"
#include "core/log/logger.h"
#include "fcstate.h"
#include <stdbool.h>
#include <stdint.h>


eSTATUS_t MC_InitAll (void) {

    eSTATUS_t status = eSTATUS_SUCCESS;

    PID_INIT (&status);
    RETURN_IF (FJ_FAIL (status), status, "Failed to init PID");

    FILTER_INIT (&status);
    RETURN_IF (FJ_FAIL (status), status, "Failed to init filter");

    // Let motors and servos be initialized by device module

    return status;
}

eSTATUS_t MC_StartAll (void) {

    eSTATUS_t status = eSTATUS_SUCCESS;

    status = PIDStart (PID_GetMutableActivePID ());
    RETURN_IF (FJ_FAIL (status), status, "Failed to start PID");

    Vec3f startingAttitude = VEC3F_ZERO ();
    status = FilterStart (Filter_GetMutableActiveFilter (), 500U, &startingAttitude);
    RETURN_IF (FJ_FAIL (status), status, "Failed to start filter");

    status = FC_SET_CURRENT_ATTITUDE (startingAttitude);
    RETURN_IF (FJ_FAIL (status), status, "Failed to set starting attitude in FC state");

    status = ActuatorsStart ();
    RETURN_IF (FJ_FAIL (status), status, "Failed to start actuators");

    return status;
}