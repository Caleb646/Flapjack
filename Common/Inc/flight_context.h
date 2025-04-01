#ifndef FLIGHT_CONTEXT_H
#define FLIGHT_CONTEXT_H

#include <stdint.h>
#include "common.h"

typedef enum {
    eFLIGHT_MODE_HELI,
    eFLIGHT_MODE_PLANE,
    eFLIGHT_MODE_PROGRAMMING
} FlightMode;

typedef struct FlightContext__ {
    // millimeters per second
    Vec3 curVel;
    Vec3 targetVel;
    // millidegrees per second (1 /1000 of a degree)
    Vec3 curAngVel;
    Vec3 targetAngVel;
    FlightMode flightMode;
} FlightContext;

void FlightContextUpdateCurrentVelocities(FlightContext *pContext, Vec3 vel, Vec3 angVel);
void FlightContextUpdateTargetVelocities(FlightContext *pContext, Vec3 vel, Vec3 angVel);
void FlightContextUpdateFlightMode(FlightContext *pContext, uint32_t flightMode);

#endif // FLIGHT_CONTEXT_H
