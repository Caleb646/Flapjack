#include "target.h"

#include "core/core.h"

#include "drivers/rx/crsf.h"
#include "drivers/rx/rx.h"

FJ_DEFINE_SHARED (Rx_t, g_Rx) = {
    .channels = {0},
    .port = {
        .cfg = {
            .id = BRD_GET_ID(RX, UART),
            .baudRate = RX_UART_BAUD_RATE,
            .irqPriority = 5,
        },
    },
};

eSTATUS_t Rx_Init (void) {

    Rx_t* pRx = &g_Rx;
    if (STATUS_FAIL (Crsf_Init (&pRx->port))) {
        LOG_ERROR ("Failed to initialize CRSF");
        return eSTATUS_FAILURE;
    }
    return eSTATUS_SUCCESS;
}

eSTATUS_t Rx_FrameUpdate (void) {

    return Crsf_ProcessFrame (g_Rx.channels);
}

eSTATUS_t Rx_Update (uint32_t usCurrentTime, uint32_t usDeltaTime) {

    return eSTATUS_SUCCESS;
}
