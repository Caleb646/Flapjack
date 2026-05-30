#include "sensors/imu.h"
#include "device/imu/imu.h"
#include "umsg_sensors.h"

eSTATUS_t SensorImu_Init(void) {
    return eSTATUS_SUCCESS;
}

eSTATUS_t SensorImu_Update(void) {
    vIMU_t* pIMU = Imu_Get();
    if (!pIMU) {
        return eSTATUS_FAILURE;
    }

    eSTATUS_t status = IMU_Update(pIMU, false, &pIMU->accelData, &pIMU->gyroData);
    if (STATUS_FAIL(status)) {
        return status;
    }

    umsg_sensors_imu_t msg = {
        .gyro        = { pIMU->gyroData.x,  pIMU->gyroData.y,  pIMU->gyroData.z  },
        .accel       = { pIMU->accelData.x, pIMU->accelData.y, pIMU->accelData.z },
        .temperature = 0.0f,
    };
    umsg_sensors_imu_publish(&msg);
    return eSTATUS_SUCCESS;
}
