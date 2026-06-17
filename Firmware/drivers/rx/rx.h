#ifndef DRIVERS_RX_RX_H
#define DRIVERS_RX_RX_H

#include "core/core.h"

#include "drivers/serial/uart.h"

/* RC channel layout produced by the receiver. */
#define RC_MAX_CHANNELS 16U
#define RC_CHANNEL_MIN  1000U
#define RC_CHANNEL_MID  1500U
#define RC_CHANNEL_MAX  2000U

#define RC_CHANNEL_IDX_ROLL     0U
#define RC_CHANNEL_IDX_PITCH    1U
#define RC_CHANNEL_IDX_YAW      2U
#define RC_CHANNEL_IDX_THROTTLE 3U
#define RC_CHANNEL_IDX_AUX_1    4U
#define RC_CHANNEL_IDX_AUX_2    5U

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