#include "target.h"

#include "core/core.h"

#include "drivers/rx/crsf.h"
#include "drivers/rx/rx.h"

FJ_DEFINE_SHARED (Rx_t, g_Rx) = {
    .channels = {0},
    .port = {
        .cfg = {
            .id = BRD_GET_UART_ID(RX),
            .baudRate = BRD_GET_BAUD_RATE(RX),
            /*
             * Above configMAX_SYSCALL_INTERRUPT_PRIORITY (5), so neither kernel
             * critical sections nor the four other priority-5 ISRs can delay it.
             * CRSF streams a byte every 24 us: with a one-byte buffer at 5 it
             * overran ~10x a second. Priority 4 and the FIFO each fixed that on
             * their own (0 overruns either way, measured on the bench); both
             * stay, for margin. The cost of 4 is that this ISR must never call
             * a FreeRTOS API - it only assembles bytes, and Rc_Task polls the
             * result. With the FIFO it could drop back to 5 if it ever needs to.
             */
            .irqPriority = 4,
            .rxFifo      = true,
        },
    },
};

eSTATUS_t Rx_Init (void) {

    Rx_t* pRx = &g_Rx;
    for (uint32_t i = 0; i < RC_MAX_CHANNELS; ++i) {
        pRx->channels[i] = RC_CHANNEL_MID;
    }
    pRx->channels[RC_CHANNEL_IDX_THROTTLE] = RC_CHANNEL_MIN;

    if (STATUS_FAIL (Crsf_Init (&pRx->port))) {
        LOG_ERROR ("Failed to initialize CRSF");
        return eSTATUS_FAILURE;
    }
    return eSTATUS_SUCCESS;
}

eSTATUS_t Rx_Update (uint32_t usCurrentTime, uint32_t usDeltaTime) {

    Rx_t* pRx = &g_Rx;

    /* A failure here is the ordinary case, not an error: this polls at 50 Hz
     * and most calls find no completed frame waiting. Only success is a
     * heartbeat. */
    eSTATUS_t const status = Crsf_ProcessFrame (pRx->channels);
    if (!STATUS_FAIL (status)) {
        pRx->usLastFrameTime = usCurrentTime;
        pRx->haveFrame       = true;
    }

    /* Unsigned difference, so this stays correct across GetMicroseconds()'
     * ~71 minute wrap. */
    bool const linkUp =
    pRx->haveFrame && ((usCurrentTime - pRx->usLastFrameTime) < RX_LINK_TIMEOUT_US);

    if (linkUp != pRx->linkUp) {
        pRx->linkUp = linkUp;
        if (linkUp) {
            LOG_INFO ("Rx: RC link up");
        } else {
            LOG_WARN ("Rx: RC link lost - holding last stick positions");
        }
    }

    return status;
}
