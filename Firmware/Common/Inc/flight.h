#ifndef FLIGHT_H
#define FLIGHT_H

#include <stdbool.h>
#include <stdint.h>

#include "core/core.h"

typedef struct FlightData_s {
    // float currentAttitude[AXIS_IDX_COUNT];
    // float targetAttitude[AXIS_IDX_COUNT];
    // float maxAttitude[AXIS_IDX_COUNT];

    bool isArmed;

    Vec3f currentAttitude; //[AXIS_IDX_COUNT];
    Vec3f targetAttitude;  //[AXIS_IDX_COUNT];
    Vec3f maxAttitude;     //[AXIS_IDX_COUNT];

    float currentAltitude;
    float targetAltitude;

    float currentThrottle;
    float targetThrottle;

} FlightData_t;

FJ_DECLARE_SHARED (FlightData_t, g_FlightData);

static inline FlightData_t* Fc_Get (void) {
    return &g_FlightData;
}

static inline bool Fc_IsArmed (void) {
    return g_FlightData.isArmed;
}


#endif