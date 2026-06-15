#include "device/mag.h"

#include "drivers/mag/magdrv.h"

#include "umsg_sensors.h"

#include "FreeRTOS.h"
#include "task.h"

#include <string.h>

eSTATUS_t Mag_Init (Mag_t* pOutSensor) {

    if (!pOutSensor) {
        return eSTATUS_NULL_ARG;
    }

    memset (pOutSensor, 0, sizeof (Mag_t));

    MagDriverConf_t conf = {
        .normalize = true,
    };

    return MagDrv_Init (&conf, &pOutSensor->drv);
}

eSTATUS_t Mag_Update (Mag_t* pSensor) {

    if (!pSensor) {
        return eSTATUS_NULL_ARG;
    }

    eSTATUS_t status = pSensor->drv.Read (pSensor->drv.ctx, false, &pSensor->field);
    if (STATUS_FAIL (status)) {
        return status;
    }

    // TODO: apply low-pass filtering (mc/filter.h). Pass-through for now.
    pSensor->fieldFiltered = pSensor->field;
    pSensor->usLastUpdate  = GetMicroseconds ();

    umsg_sensors_mag_t msg = {
        .field = { pSensor->fieldFiltered.x, pSensor->fieldFiltered.y, pSensor->fieldFiltered.z },
    };
    umsg_sensors_mag_publish (&msg);
    return eSTATUS_SUCCESS;
}

static Mag_t s_mag;

static void Mag_Task (void* args) {

    (void)args;
    while (true) {
        Mag_Update (&s_mag);
    }
}

void Mag_StartTask (uint16_t stackDepth, uint32_t priority) {

    if (STATUS_FAIL (Mag_Init (&s_mag))) {
        LOG_ERROR ("MAG unavailable; task not started");
        return;
    }
    xTaskCreate (Mag_Task, "mag", stackDepth, NULL, (UBaseType_t)priority, NULL);
}
