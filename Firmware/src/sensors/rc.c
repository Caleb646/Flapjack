#include "sensors/rc.h"
#include "drivers/rx/rx.h"
#include "fc/rc.h"
#include "umsg_rc.h"

eSTATUS_t SensorRc_Init(void) {
    return eSTATUS_SUCCESS;
}

eSTATUS_t SensorRc_Update(void) {
    uint32_t const* ch = Rx_GetChannels();
    if (!ch) {
        return eSTATUS_FAILURE;
    }

    umsg_rc_input_t msg = { .rssi = 0, .link_quality = 0 };
    for (uint8_t i = 0; i < RC_MAX_CHANNELS; i++) {
        msg.channels[i] = ch[i];
    }
    umsg_rc_input_publish(&msg);
    return eSTATUS_SUCCESS;
}
