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
    Vec3 imuUnFilteredAccel;
    Vec3 curVel;
    Vec3 targetVel;
    Vec3 pidStepVel;
    // millidegrees per second (1 /1000 of a degree)
    Vec3 imuUnFilteredGyro;
    Vec3 curAngVel;
    Vec3 targetAngVel;
    Vec3 pidStepAngVel;
    // Attitude relative to earth frame
    Vec3f attitude;
    FlightMode flightMode;
} FlightContext;

void FlightContextUpdateAttitude(FlightContext *pContext, Vec3f attitude);
void FlightContextUpdateIMUData(FlightContext *pContext, Vec3 accel, Vec3 gyro);
void FlightContextUpdateCurrentVelocities(FlightContext *pContext, Vec3 vel, Vec3 angVel);
void FlightContextUpdateTargetVelocities(FlightContext *pContext, Vec3 vel, Vec3 angVel);
void FlightContextUpdateFlightMode(FlightContext *pContext, uint32_t flightMode);

#endif // FLIGHT_CONTEXT_H
