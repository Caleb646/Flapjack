#include "tasks/baro/baro_task.h"

#include "devices/baro.h"

#include "drivers/device.h"

#include "umsg_sensors.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdbool.h>

/*
 * Fallback timeout, not the clock: with drdy live the notification lands first.
 * It stays at the old poll period - half the barometer's 50 Hz sample period -
 * so a board or an emulator that never toggles the pin degrades to exactly the
 * previous behaviour rather than stalling on portMAX_DELAY with no fault. The
 * part free-runs on its own clock, so a 20 ms poll would beat against it and
 * skip samples; checking twice per period picks each one up within 10 ms, and
 * the intervening call simply reports no new data.
 */
#define BARO_POLL_PERIOD_MS 10U

/*
 * NVIC priority for the BMP390 data-ready line. Baro_DataReady runs at it and calls
 * vTaskNotifyGiveFromISR, so it must sit at or below the kernel's syscall
 * ceiling - numerically >=, since lower numbers preempt. Asserted here because
 * this is the layer that knows: drivers/io/exti.c takes the number on faith to
 * avoid pulling FreeRTOSConfig.h into drivers/.
 */
#define BARO_DRDY_IRQ_PRIORITY 5U

STATIC_ASSERT (BARO_DRDY_IRQ_PRIORITY >= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY,
               "BARO data-ready ISR must not preempt the FreeRTOS kernel");

static Baro_t s_baro;

/* ISR context. Signal only - the SPI read happens back in the task. */
static void Baro_DataReady (void* ctx) {

    BaseType_t higherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR ((TaskHandle_t)ctx, &higherPriorityTaskWoken);
    portYIELD_FROM_ISR (higherPriorityTaskWoken);
}

void Baro_Task (void* args) {

    (void)args;

    /*
     * The notification target is this task, so Baro_Init has to run here rather
     * than from whoever created it - a task notification has no existence apart
     * from the task that receives it.
     */
    /*
     * The notification target is this task, so Baro_Init has to run here rather
     * than from whoever created it - a task notification has no existence apart
     * from the task that receives it.
     */
    DataReadySignal_t signal = {
        .Notify      = Baro_DataReady,
        .ctx         = xTaskGetCurrentTaskHandle (),
        .irqPriority = BARO_DRDY_IRQ_PRIORITY,
    };

    if (STATUS_FAIL (Baro_Init (&s_baro, &signal))) {
        LOG_ERROR ("BARO unavailable; task exiting");
        /* Not vTaskDelete: this build uses heap_1, whose vPortFree asserts -
         * and configASSERT spins with interrupts disabled, wedging the FC. */
        vTaskSuspend (NULL);
        return;
    }

    while (true) {

        (void)ulTaskNotifyTake (pdTRUE, pdMS_TO_TICKS (BARO_POLL_PERIOD_MS));

        if (STATUS_OK (Baro_Update (&s_baro))) {
            umsg_sensors_baro_t msg = {
                .pressure    = s_baro.data.pressurePa,
                .temperature = s_baro.data.temperatureC,
            };
            umsg_sensors_baro_publish (&msg);
        }
    }
}
