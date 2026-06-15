#ifndef SENSORS_IMU_H
#define SENSORS_IMU_H

#include "core/core.h"

#include "drivers/imu/imudrv.h"

typedef struct {
    ImuDriver_t drv;
    Vec3f accel;
    Vec3f gyro;
    Vec3f accelFiltered;
    Vec3f gyroFiltered;
    uint32_t usLastUpdate;
} Imu_t;

eSTATUS_t Imu_Init (Imu_t* pOutSensor);
eSTATUS_t Imu_Update (Imu_t* pOutSensor);

void Imu_StartTask (uint16_t stackDepth, uint32_t priority);

#endif // SENSORS_IMU_H
