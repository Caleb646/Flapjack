#include "devices/mag.h"

#include "drivers/mag/magdrv.h"

#include "target.h"

#include <string.h>

eSTATUS_t Mag_Init (Mag_t* pOutSensor) {

    if (!pOutSensor) {
        return eSTATUS_NULL_ARG;
    }

    memset (pOutSensor, 0, sizeof (Mag_t));

    pOutSensor->align = Align_Compose (MAG_ALIGN, CFG_BOARD_ALIGN);

    MagDriverConf_t conf = {
        .normalize = true,
    };

    return MagDrv_Init (&conf, &pOutSensor->drv);
}

eSTATUS_t Mag_Update (Mag_t* pSensor) {

    if (!pSensor) {
        return eSTATUS_NULL_ARG;
    }

    eSTATUS_t status = pSensor->drv.Read (pSensor->drv.ctx, false, &pSensor->data);
    if (STATUS_FAIL (status)) {
        return status;
    }

    // Die frame -> body FRD. No sign convention to undo: the field is a plain
    // vector, not specific force, so unlike the accel it needs only the rotation.
    Align_Apply (pSensor->align, &pSensor->data.field, &pSensor->data.field);

    // TODO: apply low-pass filtering (common/filter.h). Pass-through for now.
    pSensor->fieldFiltered = pSensor->data.field;
    pSensor->usLastUpdate  = GetMicroseconds ();

    return eSTATUS_SUCCESS;
}
