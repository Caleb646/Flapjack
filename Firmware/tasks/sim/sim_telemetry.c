#include "tasks/sim/sim_telemetry.h"

#include "core/core.h"

#include "drivers/sim_link/sim_link.h"

#include "drivers/rx/rx.h"

#include "umsg_nav.h"
#include "umsg_mission.h"

#include "FreeRTOS.h"
#include "task.h"


eSTATUS_t SimTelemetry_Init (void) {
    return SimLink_Init();
}

void SimTelemetry_Task (void* args) {

    (void)args;

    umsg_sub_handle_t nav_sub     = umsg_nav_state_subscribe (1, 1);
    umsg_sub_handle_t mission_sub = umsg_mission_state_subscribe (1, 1);

    // Latest-value caches: receive() consumes, so hold the last value for
    // iterations where nothing new arrived.
    umsg_nav_state_t     nav     = { 0 };
    umsg_mission_state_t mission = { 0 };

    for (;;) {
        umsg_nav_state_receive (nav_sub, &nav, 0);
        umsg_mission_state_receive (mission_sub, &mission, 0);

        SimLink_SendTelemetry (nav.euler, mission.armed != 0U, SimLink_GetSensorCount (), Rx_IsLinkUp ());
        vTaskDelay (pdMS_TO_TICKS (20));   // 50 Hz
    }
}
