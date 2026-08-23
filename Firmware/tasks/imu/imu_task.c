#include "tasks/imu/imu_task.h"

#include "devices/imu.h"

#include "drivers/device.h"

#include "umsg_sensors.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdbool.h>

/*
 * NVIC priority for the BMI323 data-ready line. Imu_DataReady runs at it and
 * calls vTaskNotifyGiveFromISR, so it must sit at or below the kernel's syscall
 * ceiling - numerically >=, since lower numbers preempt. A higher-priority ISR
 * calling a FreeRTOS API preempts the kernel's own critical sections and
 * corrupts it intermittently, which is why this is asserted rather than trusted.
 *
 * The assert lives here because this is the layer that knows: drivers/io/exti.c
 * takes the number on faith to avoid pulling FreeRTOSConfig.h into drivers/.
 */
#define IMU_DRDY_IRQ_PRIORITY 5U

STATIC_ASSERT (IMU_DRDY_IRQ_PRIORITY >= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY,
               "IMU data-ready ISR must not preempt the FreeRTOS kernel");

/*
 * Never longer than one 400 Hz sample period plus slack. This is a watchdog,
 * not the clock: with the interrupt live the notification almost always lands
 * first. It stays at the old poll period so a board or an emulator that never
 * toggles the pin degrades to exactly the previous behaviour - a 500 Hz poll -
 * instead of stalling on portMAX_DELAY with no fault.
 */
#define IMU_DRDY_TIMEOUT_MS 2U

static Imu_t s_imu;

/* ISR context. Signal only - the SPI read happens back in the task. */
static void Imu_DataReady (void* ctx) {

    BaseType_t higherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR ((TaskHandle_t)ctx, &higherPriorityTaskWoken);
    portYIELD_FROM_ISR (higherPriorityTaskWoken);
}

void Imu_Task (void* args) {

    (void)args;

    /*
     * The notification target is this task, so Imu_Init has to run here rather
     * than from whoever created it - a task notification has no existence apart
     * from the task that receives it.
     */
    DataReadySignal_t signal = {
        .Notify      = Imu_DataReady,
        .ctx         = xTaskGetCurrentTaskHandle (),
        .irqPriority = IMU_DRDY_IRQ_PRIORITY,
    };

    if (STATUS_FAIL (Imu_Init (&s_imu, &signal))) {
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

        (void)ulTaskNotifyTake (pdTRUE, pdMS_TO_TICKS (IMU_DRDY_TIMEOUT_MS));

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
    }
}
