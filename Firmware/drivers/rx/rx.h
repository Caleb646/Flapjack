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

/*
 * How long without a valid RC frame before the link counts as lost. The CRSF
 * spec says a receiver in failsafe simply stops transmitting, and recommends
 * the FC waits one second before reacting.
 */
#define RX_LINK_TIMEOUT_US 1000000U

typedef struct {
    uint32_t channels[RC_MAX_CHANNELS];
    UartPort_t port;
    // Timestamp of the last frame that decoded cleanly, and whether one has ever
    // arrived - so a freshly booted FC reads as link-DOWN rather than link-up.
    uint32_t usLastFrameTime;
    bool haveFrame;
    /*
     * volatile: in a dual-core build Rx_Task writes this on CM4 while Rc_Task
     * polls it on CM7 through shared memory, the same arrangement as
     * s_IsCM4Ready in main.c.
     */
    bool volatile linkUp;
} Rx_t;

FJ_DECLARE_SHARED (Rx_t, g_Rx);

eSTATUS_t Rx_Init (void);
eSTATUS_t Rx_Update (uint32_t usCurrentTime, uint32_t usDeltaTime);

static inline uint32_t const* Rx_GetChannels (void) {
    return g_Rx.channels;
}

/*
 * True while RC frames are still arriving. Reporting only - nothing acts on it
 * yet, so a lost link leaves the vehicle flying its last commanded rates. The
 * channels themselves are NOT scrubbed when this goes false; deciding what a
 * failsafe should actually do is a separate piece of work, and a controlled
 * descent needs an altitude estimate the navigation filter does not produce.
 */
static inline bool Rx_IsLinkUp (void) {
    return g_Rx.linkUp;
}


#endif /* DRIVERS_RX_RX_H */