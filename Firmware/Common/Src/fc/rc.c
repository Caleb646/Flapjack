#include "fc/rc.h"

#include "drivers/rx/rx.h"

#include "mc/motors.h"

eSTATUS_t Rc_Init (void) {

    return eSTATUS_SUCCESS;
}

eSTATUS_t Rc_Update (uint32_t usCurrentTime, uint32_t usDeltaTime) {

    uint32_t const* pChannels = Rx_GetChannels ();
    if (!pChannels) {
        return eSTATUS_FAILURE;
    }

    // TODO: update flight mode, target attitude, and target throttle based on RC channels
    // TODO: check if system should be armed. If should be armed then arm motors
    return eSTATUS_SUCCESS;
}