#ifndef SENSORS_BARO_H
#define SENSORS_BARO_H

#include "core/core.h"

#include "drivers/baro/barodrv.h"

typedef struct {
    BaroDriver_t drv;
    BaroData_t data;
    uint32_t usLastUpdate;
} Baro_t;

eSTATUS_t Baro_Init (Baro_t* pOutSensor);
eSTATUS_t Baro_Update (Baro_t* pSensor);

#endif // SENSORS_BARO_H
