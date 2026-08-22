#include "tasks/mag/mag_task.h"

#include "devices/mag.h"

#include "umsg_sensors.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdbool.h>

/*
 * Heading is the only consumer and it moves slowly, so 100 Hz is well clear of
 * what the estimate needs. Without a limit this task reads as fast as SPI5
 * allows, which saturates the bus the barometer shares and burns a core at the
 * lowest priority for samples nothing asks for.
 */
#define MAG_POLL_PERIOD_MS 10U

static Mag_t s_mag;

void Mag_Task (void* args) {

    (void)args;

    if (STATUS_FAIL (Mag_Init (&s_mag))) {
        LOG_ERROR ("MAG unavailable; task exiting");
        /* Not vTaskDelete: this build uses heap_1, whose vPortFree asserts -
         * and configASSERT spins with interrupts disabled, wedging the FC. */
        vTaskSuspend (NULL);
        return;
    }

    while (true) {
        if (STATUS_OK (Mag_Update (&s_mag))) {
            umsg_sensors_mag_t msg = {
                .field = { s_mag.fieldFiltered.x, s_mag.fieldFiltered.y, s_mag.fieldFiltered.z },
            };
            umsg_sensors_mag_publish (&msg);
        }
        vTaskDelay (pdMS_TO_TICKS (MAG_POLL_PERIOD_MS));
    }
}
