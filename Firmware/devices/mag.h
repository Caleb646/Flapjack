#ifndef SENSORS_MAG_H
#define SENSORS_MAG_H

#include "core/core.h"

#include "drivers/mag/magdrv.h"

typedef struct {
    MagDriver_t drv;
    Vec3f field;
    Vec3f fieldFiltered;
    uint32_t usLastUpdate;
} Mag_t;

eSTATUS_t Mag_Init (Mag_t* pOutSensor);
eSTATUS_t Mag_Update (Mag_t* pOutSensor);

#endif // SENSORS_MAG_H
