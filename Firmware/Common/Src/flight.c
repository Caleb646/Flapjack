#include "flight.h"

#include "core/core.h"

FJ_DEFINE_SHARED (FlightData_t, g_FlightData) = {
    .max = { [AXIS_IDX_ROLL] = 45.0F, [AXIS_IDX_PITCH] = 45.0F, [AXIS_IDX_YAW] = 180.0F, [AXIS_IDX_THROTTLE] = 1.0F },
};