#ifndef DRIVERS_RX_RX_H
#define DRIVERS_RX_RX_H

#include "fc/rc.h"

#include "core/core.h"

#include "peripheral/bus/uart.h"


typedef struct {
    uint32_t channels[RC_MAX_CHANNELS];
    UartPort_t port;
} Rx_t;

FJ_DECLARE_SHARED (Rx_t, g_Rx);

eSTATUS_t Rx_Init (void);
eSTATUS_t Rx_Update (uint32_t usCurrentTime, uint32_t usDeltaTime);

static inline uint32_t const* Rx_GetChannels (void) {
    return g_Rx.channels;
}


#endif /* DRIVERS_RX_RX_H */