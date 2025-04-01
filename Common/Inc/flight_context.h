#ifndef FLIGHT_CONTEXT_H
#define FLIGHT_CONTEXT_H

#include <stdint.h>
#include "common.h"

typedef enum {
    eFLIGHT_MODE_HELI,
    eFLIGHT_MODE_PLANE
} FlightMode;

typedef struct {
    // millimeters per second
    UVec3 vel;
    // millidegrees per second (1 /1000 of a degree)
    UVec3 angVel;
    // helicoptor or plane
    FlightMode flightMode;
} FlightContext;

void FlightContextUpdateVelocities(FlightContext *pContext, UVec3 vel, UVec3 angVel);
void FlightContextUpdateFlightMode(FlightContext *pContext, uint32_t flightMode);

#endif // FLIGHT_CONTEXT_H
