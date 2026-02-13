#ifndef FLIGHT_H
#define FLIGHT_H

#include <stdbool.h>
#include <stdint.h>

#include "core/core.h"

typedef struct FlightData_s {

    bool isArmed;

    float current[AXIS_IDX_COUNT];
    float target[AXIS_IDX_COUNT];
    float max[AXIS_IDX_COUNT];

    float currentAltitude;
    float targetAltitude;

} FlightData_t;

FJ_DECLARE_SHARED (FlightData_t, g_FlightData);

static inline FlightData_t* Fc_Get (void) {
    return &g_FlightData;
}

static inline bool Fc_IsArmed (void) {
    return g_FlightData.isArmed;
}


#endif