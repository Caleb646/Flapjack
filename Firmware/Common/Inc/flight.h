#ifndef FLIGHT_H
#define FLIGHT_H

#include "core/core.h"

typedef struct FlightData_s {
    // float currentAttitude[AXIS_IDX_COUNT];
    // float targetAttitude[AXIS_IDX_COUNT];
    // float maxAttitude[AXIS_IDX_COUNT];

    Vec3f currentAttitude; //[AXIS_IDX_COUNT];
    Vec3f targetAttitude;  //[AXIS_IDX_COUNT];
    Vec3f maxAttitude;     //[AXIS_IDX_COUNT];

    float currentAltitude;
    float targetAltitude;

    float currentThrottle;
    float targetThrottle;
} FlightData_t;

FJ_DECLARE_SHARED (FlightData_t, g_FlightData);


#endif