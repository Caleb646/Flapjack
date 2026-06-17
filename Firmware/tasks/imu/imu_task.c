#include "tasks/imu/imu_task.h"

#include "devices/imu.h"

#include "umsg_sensors.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdbool.h>

static Imu_t s_imu;

void Imu_Task (void* args) {

    (void)args;

    if (STATUS_FAIL (Imu_Init (&s_imu))) {
        LOG_ERROR ("IMU unavailable; task exiting");
        vTaskDelete (NULL);
        return;
    }

    while (true) {
        if (STATUS_OK (Imu_Update (&s_imu))) {
            umsg_sensors_imu_t msg = {
                .gyro        = { s_imu.gyroFiltered.x,  s_imu.gyroFiltered.y,  s_imu.gyroFiltered.z },
                .accel       = { s_imu.accelFiltered.x, s_imu.accelFiltered.y, s_imu.accelFiltered.z },
                .temperature = 0.0f,
            };
            umsg_sensors_imu_publish (&msg);
        }
    }
}
