#ifndef FLIGHT_CONTEXT_H
#define FLIGHT_CONTEXT_H

#include <stdint.h>
#include "common.h"

#define FLIGHT_CONTEXT_MAX_THROTTLE 1.0f
#define FLIGHT_CONTEXT_MIN_THROTTLE 0.0f

typedef enum {
    eFLIGHT_MODE_HELI,
    eFLIGHT_MODE_PLANE,
    eFLIGHT_MODE_PROGRAMMING
} FlightMode;

typedef struct FlightContext__ {
    // millimeters per second ^ 2
    Vec3 imuUnFilteredAccel;
    // millimeters per second
    Vec3 currentVel;
    // Vec3 targetVel;
    // Vec3 pidStepVel;
    // millidegrees per second (1 /1000 of a degree)
    Vec3 imuUnFilteredGyro;
    Vec3 currentAngVel;
    // Vec3 targetAngVel;
    // Vec3 pidStepAngVel;
    // Attitude relative to earth frame
    // in millidegrees
    Vec3f currentAttitude;
    Vec3f targetAttitude;
    // percent in range 0 to 1
    float currentThrottle;
    float targetThrottle;
    FlightMode flightMode;
} FlightContext;

void FlightContextUpdateCurrentAttitude(FlightContext *pContext, Vec3f attitude);
void FlightContextUpdateTargetAttitude(FlightContext *pContext, Vec3f attitude);
void FlightContextUpdateIMUData(FlightContext *pContext, Vec3 accel, Vec3 gyro);
void FlightContextUpdateFlightMode(FlightContext *pContext, uint32_t flightMode);
// void FlightContextUpdateCurrentVelocities(FlightContext *pContext, Vec3 vel, Vec3 angVel);
// void FlightContextUpdateTargetVelocities(FlightContext *pContext, Vec3 vel, Vec3 angVel);


#endif // FLIGHT_CONTEXT_H
