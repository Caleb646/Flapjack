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
        LOG_ERROR ("Imu unavailable; task exiting");
        /* Not vTaskDelete: this build uses heap_1, whose vPortFree asserts -
         * and configASSERT spins with interrupts disabled, wedging the FC. */
        vTaskSuspend (NULL);
        return;
    }

    LOG_INFO ("Imu initialized successfully");
    /*
     * Running total, not a per-sample tick: a subscriber that reads slower than
     * this task publishes still recovers the true count from whichever message
     * it happens to see, so the pacing metric does not depend on anyone's queue
     * being deep enough.
     */
    umsg_sensors_imu_status_t status = { 0 };

    while (true) {
        if (STATUS_OK (Imu_Update (&s_imu))) {
            umsg_sensors_imu_t msg = {
                .gyro        = { s_imu.gyroFiltered.x,  s_imu.gyroFiltered.y,  s_imu.gyroFiltered.z },
                .accel       = { s_imu.accelFiltered.x, s_imu.accelFiltered.y, s_imu.accelFiltered.z },
                .temperature = 0.0f,
            };
            umsg_sensors_imu_publish (&msg);

            status.sample_count++;
            umsg_sensors_imu_status_publish (&status);
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
