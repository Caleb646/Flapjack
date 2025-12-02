#ifndef DRIVERS_SENSORS_INERTIAL_IMU_DEFS_H
#define DRIVERS_SENSORS_INERTIAL_IMU_DEFS_H

#include <stdint.h>

#include "drivers/sensors/cns_defs.h"

#include "drivers/sensors/inertial/acc_defs.h"
#include "drivers/sensors/inertial/gyro_defs.h"

#include "drivers/bus/bus_defs.h"

typedef uint8_t IMU_INTERFACE_ID_t;
enum {
    IMU_INTERFACE_ID_NULL = 0,

    IMU_INTERFACE_BMI323 = 1,
};

typedef struct IMUDevice_s {
    NAV_SENSOR_ID_t sensorId;
    IMU_INTERFACE_ID_t interfaceId;
    BusDevice_t* pBusDevice;
    IACCDevice_t acc;
    IGYRODevice_t gyro;
} IMUDevice_t;


#endif // DRIVERS_SENSORS_INERTIAL_IMU_DEFS_H