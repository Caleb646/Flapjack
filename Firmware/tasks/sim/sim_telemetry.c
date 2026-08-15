#include "tasks/sim/sim_telemetry.h"

#include "core/core.h"

#include "drivers/sim_link/sim_link.h"

#include "umsg_nav.h"
#include "umsg_mission.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdbool.h>

void SimTelemetry_Task (void* args) {

    (void)args;
    for (;;) {
        float euler[3] = { 0.0F, 0.0F, 0.0F };
        umsg_nav_state_t nav;
        if (umsg_nav_state_peek (&nav)) {
            euler[0] = nav.euler[0];
            euler[1] = nav.euler[1];
            euler[2] = nav.euler[2];
        }

        bool armed = false;
        umsg_mission_state_t mission;
        if (umsg_mission_state_peek (&mission)) {
            armed = mission.armed != 0U;
        }

        SimLink_SendTelemetry (euler, armed, SimLink_GetSensorCount ());
        vTaskDelay (pdMS_TO_TICKS (20));   // 50 Hz
    }
}
