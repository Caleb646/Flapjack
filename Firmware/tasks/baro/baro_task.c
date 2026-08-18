#include "tasks/baro/baro_task.h"

#include "devices/baro.h"

#include "umsg_sensors.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdbool.h>

static Baro_t s_baro;

void Baro_Task (void* args) {

    (void)args;

    if (STATUS_FAIL (Baro_Init (&s_baro))) {
        LOG_ERROR ("BARO unavailable; task exiting");
        vTaskDelete (NULL);
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
    }
}
