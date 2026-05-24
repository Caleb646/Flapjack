#include "fc/rc.h"

#include "drivers/rx/rx.h"

#include "flight.h"
#include "mc/motors.h"

eSTATUS_t Rc_Init (void) {

    return eSTATUS_SUCCESS;
}

eSTATUS_t Rc_Update (uint32_t usCurrentTime, uint32_t usDeltaTime) {

    uint32_t const* pChannels = Rx_GetChannels ();
    if (!pChannels) {
        return eSTATUS_FAILURE;
    }

    // Arm/disarm from Aux1 switch (high > 1750 = arm, low < 1250 = disarm)
    Flight_t* pFlight     = Fc_Get ();
    uint32_t  aux1        = pChannels[RC_CHANNEL_IDX_AUX_1];

    if (aux1 > 1750U && !pFlight->isArmed) {
        if (STATUS_OK (Motors_Arm ())) {
            pFlight->isArmed = true;
            LOG_INFO ("RC: armed");
        }
    } else if (aux1 < 1250U && pFlight->isArmed) {
        Motors_Disarm ();
        pFlight->isArmed = false;
        LOG_INFO ("RC: disarmed");
    }

    // TODO: update flight mode, target attitude, and target throttle based on RC channels
    return eSTATUS_SUCCESS;
}