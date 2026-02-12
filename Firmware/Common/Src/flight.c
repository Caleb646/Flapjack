#include "flight.h"

#include "core/core.h"

FJ_DEFINE_SHARED (FlightData_t, g_FlightData) = {
    .currentAttitude = { { 0.0F } },
    .targetAttitude  = { { 0.0F } },
    .maxAttitude     = { .roll = 45.0F, .pitch = 45.0F, .yaw = 180.0F },
    .currentAltitude = 0.0F,
    .targetAltitude  = 0.0F,
    .currentThrottle = 0.0F,
    .targetThrottle  = 0.0F,
};