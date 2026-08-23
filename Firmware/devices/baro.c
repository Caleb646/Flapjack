#include "devices/baro.h"

#include "drivers/baro/barodrv.h"

#include <string.h>

eSTATUS_t Baro_Init (Baro_t* pOutSensor, DataReadySignal_t const* pSignal) {

    if (!pOutSensor) {
        return eSTATUS_NULL_ARG;
    }

    memset (pOutSensor, 0, sizeof (Baro_t));

    if (pSignal) {
        pOutSensor->drv.cfg.signal = *pSignal;
    }

    return BaroDrv_Init (&pOutSensor->drv);
}

eSTATUS_t Baro_Update (Baro_t* pSensor) {

    if (!pSensor) {
        return eSTATUS_NULL_ARG;
    }

    eSTATUS_t status =
    pSensor->drv.Read (pSensor->drv.ctx, false, &pSensor->data);
    if (STATUS_FAIL (status)) {
        return status;
    }

    pSensor->usLastUpdate = GetMicroseconds ();
    return eSTATUS_SUCCESS;
}
