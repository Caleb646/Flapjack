#ifndef DRIVERS_SENSORS_INERTIAL_GYRO_DEFS_H
#define DRIVERS_SENSORS_INERTIAL_GYRO_DEFS_H

#include <stdint.h>

#include "drivers/sensors/cns_defs.h"

#include "drivers/bus/bus_defs.h"


typedef struct IGYRODevice_s {
    float scaleFactor;
    uint16_t sampleRateHz;
} IGYRODevice_t;

typedef struct GYRODevice_s {
    InertialDevice_t inertial;
    IGYRODevice_t i;
} GYRODevice_t;

#endif // DRIVERS_SENSORS_INERTIAL_GYRO_DEFS_H