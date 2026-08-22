#include "tasks/imu/imu_task.h"

#include "devices/imu.h"

#include "umsg_sensors.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdbool.h>

static Imu_t s_imu;

void Imu_Task (void* args) {

    (void)args;

    /*
     * Keep trying rather than giving up: the part does not have to be ready the
     * first time this runs. Under Renode the sample source may not be connected
     * yet, and on hardware the part may still be coming out of reset.
     *
     * 20 ms between attempts, but the attempt itself sets the cadence: a failing
     * IMUSoftReset spends ~2.5 s in Delay() waiting on the feature engine, so
     * retries land about every 2.7 s. Delay() blocks rather than spins once the
     * scheduler is up, so that wait no longer costs anything but time.
     */
    while (STATUS_FAIL (Imu_Init (&s_imu))) {
        vTaskDelay (pdMS_TO_TICKS (20));
    }

    LOG_INFO ("Imu initialized successfully");
    while (true) {
        if (STATUS_OK (Imu_Update (&s_imu))) {
            umsg_sensors_imu_t msg = {
                .gyro        = { s_imu.gyroFiltered.x,  s_imu.gyroFiltered.y,  s_imu.gyroFiltered.z },
                .accel       = { s_imu.accelFiltered.x, s_imu.accelFiltered.y, s_imu.accelFiltered.z },
                .temperature = 0.0f,
            };
            umsg_sensors_imu_publish (&msg);
        }

        /*
         * Cap the loop at 500 Hz. The BMI323 path has no data-ready wait - it
         * polls BMI3_REG_STATUS in a tight loop - so without this the task
         * never blocks, and at TASK_PRIORITY_SENSOR_IMU it starves every
         * priority-1 task under it: the log drain, mag and baro included.
         * configTICK_RATE_HZ is 1000, so 2 ms is exactly 2 ticks.
         */
        vTaskDelay (pdMS_TO_TICKS (2));
    }
}
