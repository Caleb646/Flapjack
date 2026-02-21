#include "flight.h"

#include "core/core.h"

FJ_DEFINE_SHARED (FlightData_t, g_FlightData) = {
    .max = { [AXIS_IDX_ROLL] = 45.0F, [AXIS_IDX_PITCH] = 45.0F, [AXIS_IDX_YAW] = 180.0F, [AXIS_IDX_THROTTLE] = 1.0F },
};

void Fc_LogData_ (FlightData_t* pFlightData) {

    LOG_4_FLOATS (
    LOG_DATA_TYPE_ATTITUDE,
    roll,
    pFlightData->current[AXIS_IDX_ROLL],
    pitch,
    pFlightData->current[AXIS_IDX_PITCH],
    yaw,
    pFlightData->current[AXIS_IDX_YAW],
    throttle,
    pFlightData->current[AXIS_IDX_THROTTLE]
    );
}