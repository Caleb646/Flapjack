#ifndef TASKS_SIM_SIM_TELEMETRY_H
#define TASKS_SIM_SIM_TELEMETRY_H

#include "core/core.h"

/* Periodically downlinks nav attitude + armed + IMU pacing count over the sim
 * link, for open-loop validation. Registered on CM7 only in the HIL build. */
void SimTelemetry_Task (void* args);

#endif // TASKS_SIM_SIM_TELEMETRY_H
