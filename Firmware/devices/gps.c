#include "core/core.h"

#include "devices/gps.h"

#include "drivers/gps/gpsdrv.h"

#include <string.h>

eSTATUS_t Gps_Init(Gps_t* pOutGps, DataReadySignal_t const* pSignal) {

    if (!pOutGps) {
        return eSTATUS_NULL_ARG;
    }

    memset(pOutGps, 0, sizeof(Gps_t));

    if (pSignal) {
        pOutGps->driver.cfg.signal = *pSignal;
    }

    eSTATUS_t status = GpsDrv_Init(&pOutGps->driver);
    if (STATUS_FAIL(status)) {
        LOG_ERROR ("Failed to initialize GPS driver");
        return status;
    }

    return eSTATUS_SUCCESS;
}

eSTATUS_t Gps_Update(Gps_t* pGps) {

    if (!pGps) {
        return eSTATUS_NULL_ARG;
    }

    if(!pGps->driver.IsDataReady(pGps->driver.ctx)) {
        return eSTATUS_BUSY;
    }

    return pGps->driver.Read(pGps->driver.ctx, false, &pGps->data);
}

bool Gps_HasFix(Gps_t const* pGps) {

    if (!pGps || pGps->data.fixType == 0U) {
        return false;
    }

    /* Unsigned subtraction, deliberately: a uint32_t of microseconds wraps every
     * ~71.6 minutes, and (now - then) stays correct across the wrap where an
     * absolute comparison would not. */
    return (GetMicroseconds() - pGps->data.usLastFix) < GPS_FIX_TIMEOUT_US;
}
