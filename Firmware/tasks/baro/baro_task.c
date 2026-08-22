#include "tasks/baro/baro_task.h"

#include "devices/baro.h"

#include "umsg_sensors.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdbool.h>

/*
 * Half the barometer's 50 Hz sample period. The part free-runs on its own clock
 * rather than this task's, so sampling at the same 20 ms period would beat
 * against it and skip samples; checking twice per period picks each one up
 * within 10 ms, and the intervening call simply reports no new data.
 */
#define BARO_POLL_PERIOD_MS 10U

static Baro_t s_baro;

void Baro_Task (void* args) {

    (void)args;

    if (STATUS_FAIL (Baro_Init (&s_baro))) {
        LOG_ERROR ("BARO unavailable; task exiting");
        /* Not vTaskDelete: this build uses heap_1, whose vPortFree asserts -
         * and configASSERT spins with interrupts disabled, wedging the FC. */
        vTaskSuspend (NULL);
        return;
    }

    while (true) {
        if (STATUS_OK (Baro_Update (&s_baro))) {
            umsg_sensors_baro_t msg = {
                .pressure    = s_baro.data.pressurePa,
                .temperature = s_baro.data.temperatureC,
            };
            umsg_sensors_baro_publish (&msg);
        }
        vTaskDelay (pdMS_TO_TICKS (BARO_POLL_PERIOD_MS));
    }
}
