#ifndef DRIVERS_SENSORS_INERTIAL_ACC_DEFS_H
#define DRIVERS_SENSORS_INERTIAL_ACC_DEFS_H


#include <stdint.h>

#include "drivers/sensors/cns_defs.h"

#include "drivers/bus/bus_defs.h"


typedef uint8_t ACC_INTERFACE_ID_t;
enum {
    ACC_INTERFACE_ID_NULL = 0,

    // ACC_INTERFACE_BMI323 = 1,
};

typedef struct IACCDevice_s {
    float scaleFactor;
    uint16_t sampleRateHz;
} IACCDevice_t;

typedef struct ACCDevice_s {
    NAV_SENSOR_ID_t sensorId;
    ACC_INTERFACE_ID_t interfaceId;
    BusDevice_t busDevice;
    IACCDevice_t i;
} ACCDevice_t;

#endif // DRIVERS_SENSORS_INERTIAL_ACC_DEFS_H