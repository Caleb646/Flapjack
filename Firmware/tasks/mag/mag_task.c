#include "tasks/mag/mag_task.h"

#include "devices/mag.h"

#include "umsg_sensors.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdbool.h>

static Mag_t s_mag;

void Mag_Task (void* args) {

    (void)args;

    if (STATUS_FAIL (Mag_Init (&s_mag))) {
        LOG_ERROR ("MAG unavailable; task exiting");
        vTaskDelete (NULL);
        return;
    }

    while (true) {
        if (STATUS_OK (Mag_Update (&s_mag))) {
            umsg_sensors_mag_t msg = {
                .field = { s_mag.fieldFiltered.x, s_mag.fieldFiltered.y, s_mag.fieldFiltered.z },
            };
            umsg_sensors_mag_publish (&msg);
        }
    }
}
