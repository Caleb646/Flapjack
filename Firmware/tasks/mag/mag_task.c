#include "tasks/mag/mag_task.h"

#include "devices/mag.h"

#include "drivers/device.h"

#include "umsg_sensors.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdbool.h>

/*
 * Fallback timeout, not the clock: with MEAS_M_DONE wired to the INT pin the
 * notification lands first. It stays at the old poll period so a board or an
 * emulator that never toggles the pin degrades to exactly the previous
 * behaviour rather than stalling on portMAX_DELAY with no fault.
 *
 * It matches the part's own rate on purpose. Heading is the only consumer and
 * it moves slowly, so mmc5983.c runs the die at 100 Hz and every drdy is a
 * sample this task actually wants - no polling beat, and nothing spent on the
 * SPI5 bus the barometer shares for samples nobody asks for.
 */
#define MAG_POLL_PERIOD_MS 10U

/*
 * NVIC priority for the MMC5983 data-ready line. Mag_DataReady runs at it and
 * calls vTaskNotifyGiveFromISR, so it must sit at or below the kernel's syscall
 * ceiling - numerically >=, since lower numbers preempt. Asserted here because
 * this is the layer that knows: drivers/io/exti.c takes the number on faith to
 * avoid pulling FreeRTOSConfig.h into drivers/.
 */
#define MAG_DRDY_IRQ_PRIORITY 5U

STATIC_ASSERT (MAG_DRDY_IRQ_PRIORITY >= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY,
               "MAG data-ready ISR must not preempt the FreeRTOS kernel");

static Mag_t s_mag;

/* ISR context. Signal only - the SPI read happens back in the task. */
static void Mag_DataReady (void* ctx) {

    BaseType_t higherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR ((TaskHandle_t)ctx, &higherPriorityTaskWoken);
    portYIELD_FROM_ISR (higherPriorityTaskWoken);
}

void Mag_Task (void* args) {

    (void)args;

    /*
     * The notification target is this task, so Mag_Init has to run here rather
     * than from whoever created it - a task notification has no existence apart
     * from the task that receives it.
     */
    /*
     * The notification target is this task, so Mag_Init has to run here rather
     * than from whoever created it - a task notification has no existence apart
     * from the task that receives it.
     */
    DataReadySignal_t signal = {
        .Notify      = Mag_DataReady,
        .ctx         = xTaskGetCurrentTaskHandle (),
        .irqPriority = MAG_DRDY_IRQ_PRIORITY,
    };

    if (STATUS_FAIL (Mag_Init (&s_mag, &signal))) {
        LOG_ERROR ("MAG unavailable; task exiting");
        /* Not vTaskDelete: this build uses heap_1, whose vPortFree asserts -
         * and configASSERT spins with interrupts disabled, wedging the FC. */
        vTaskSuspend (NULL);
        return;
    }

    while (true) {

        (void)ulTaskNotifyTake (pdTRUE, pdMS_TO_TICKS (MAG_POLL_PERIOD_MS));

        if (STATUS_OK (Mag_Update (&s_mag))) {
            umsg_sensors_mag_t msg = {
                .field = { s_mag.fieldFiltered.x, s_mag.fieldFiltered.y, s_mag.fieldFiltered.z },
            };
            umsg_sensors_mag_publish (&msg);
        }
    }
}
