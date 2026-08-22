#ifndef SENSORS_IMU_H
#define SENSORS_IMU_H

#include "common/align.h"

#include "core/core.h"

#include "drivers/imu/imudrv.h"

typedef struct {
    ImuDriver_t drv;
    // Die-to-body rotation: IMU_ALIGN composed with CFG_BOARD_ALIGN, fixed at init.
    eSensorAlign_t align;
    ImuData_t data;
    Vec3f accelFiltered;
    Vec3f gyroFiltered;
    uint32_t usLastUpdate;
} Imu_t;

eSTATUS_t Imu_Init (Imu_t* pOutSensor);
eSTATUS_t Imu_Update (Imu_t* pOutSensor);

#endif // SENSORS_IMU_H
