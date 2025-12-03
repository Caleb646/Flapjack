#ifndef DRIVERS_SENSORS_INERTIAL_IMU_DEFS_H
#define DRIVERS_SENSORS_INERTIAL_IMU_DEFS_H

#include <stdint.h>

#include "drivers/sensors/cns_defs.h"

#include "drivers/sensors/inertial/acc_defs.h"
#include "drivers/sensors/inertial/gyro_defs.h"

#include "drivers/bus/bus_defs.h"


typedef struct IMUDevice_s {
    InertialDevice_t inertial;
    IACCDevice_t acc;
    IGYRODevice_t gyro;
} IMUDevice_t;


#endif // DRIVERS_SENSORS_INERTIAL_IMU_DEFS_H