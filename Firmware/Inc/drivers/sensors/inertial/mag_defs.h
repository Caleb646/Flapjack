#ifndef DRIVERS_SENSORS_INERTIAL_MAG_DEFS_H
#define DRIVERS_SENSORS_INERTIAL_MAG_DEFS_H

#include <stdint.h>

#include "drivers/sensors/cns_defs.h"


typedef struct IMAGDevice_s {
    float scaleFactor;
    uint16_t sampleRateHz;
} IMAGDevice_t;

typedef struct MAGDevice_s {
    InertialDevice_t inertial;
    IMAGDevice_t i;
} MAGDevice_t;

#endif // DRIVERS_SENSORS_INERTIAL_MAG_DEFS_H