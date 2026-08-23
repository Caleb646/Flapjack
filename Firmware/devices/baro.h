#ifndef SENSORS_BARO_H
#define SENSORS_BARO_H

#include "core/core.h"

#include "drivers/baro/barodrv.h"

typedef struct {
    BaroDriver_t drv;
    BaroData_t data;
    uint32_t usLastUpdate;
} Baro_t;

/* pSignal may be NULL, in which case Baro_Update polls the part on its own. */
eSTATUS_t Baro_Init (Baro_t* pOutSensor, DataReadySignal_t const* pSignal);
eSTATUS_t Baro_Update (Baro_t* pSensor);

#endif // SENSORS_BARO_H
