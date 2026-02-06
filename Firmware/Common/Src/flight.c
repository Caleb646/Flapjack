#include "flight.h"

#include "core/core.h"

FJ_DEFINE_SHARED (FlightData_t, g_FlightData) = {
    .currentAttitude = { 0.0F },
    .targetAttitude  = { 0.0F },
    .maxAttitude     = { 
        [AXIS_IDX_ROLL]  = 45.0F,
        [AXIS_IDX_PITCH] = 45.0F,
        [AXIS_IDX_YAW]   = 180.0F,
    },
    .currentAltitude = 0.0F,
    .targetAltitude  = 0.0F,
    .currentThrottle = 0.0F,
    .targetThrottle  = 0.0F,
};