#ifndef DRIVERS_SENSORS_INERTIAL_GYRO_DEFS_H
#define DRIVERS_SENSORS_INERTIAL_GYRO_DEFS_H

#include <stdint.h>

#include "drivers/sensors/cns_defs.h"

#include "drivers/bus/bus_defs.h"

typedef uint8_t GYRO_INTERFACE_ID_t;
enum {
    GYRO_INTERFACE_ID_NULL = 0,

    // GYRO_INTERFACE_BMI323 = 1,
};

typedef struct IGYRODevice_s {
    float scaleFactor;
    uint16_t sampleRateHz;
} IGYRODevice_t;

typedef struct GYRODevice_s {
    NAV_SENSOR_ID_t sensorId;
    GYRO_INTERFACE_ID_t interfaceId;
    BusDevice_t busDevice;
    IGYRODevice_t i;
} GYRODevice_t;

#endif // DRIVERS_SENSORS_INERTIAL_GYRO_DEFS_H