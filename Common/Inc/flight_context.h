#ifndef FLIGHT_CONTEXT_H
#define FLIGHT_CONTEXT_H

#include "common.h"
#include <stdint.h>

#define FLIGHT_CONTEXT_MAX_THROTTLE 1.0f
#define FLIGHT_CONTEXT_MIN_THROTTLE 0.0f

typedef enum {
    eFLIGHT_MODE_HELI,
    eFLIGHT_MODE_PLANE,
    eFLIGHT_MODE_PROGRAMMING
} FlightMode;

typedef struct FlightContext__ {
    // meters per second ^ 2
    Vec3f imuUnFilteredAccel;
    // Vec3 currentVel;
    // Vec3 targetVel;
    // Vec3 pidStepVel;
    // degrees per second
    Vec3f imuUnFilteredGyro;
    // Vec3 currentAngVel;
    // Vec3 targetAngVel;
    // Vec3 pidStepAngVel;
    // Attitude relative to earth frame
    // In degrees
    Vec3f currentAttitude;
    Vec3f targetAttitude;
    Vec3f pidAttitude;
    Vec3f maxAttitude;
    // percent in range 0 to 1
    float currentThrottle;
    float targetThrottle;
    FlightMode flightMode;
} FlightContext;

void FlightContextUpdateCurrentAttitude (FlightContext* pContext, Vec3f attitude);
void FlightContextUpdateTargetAttitudeThrottle (FlightContext* pContext, Vec3f attitude, float throttle);
void FlightContextUpdatePIDAttitude (FlightContext* pContext, Vec3f attitude);
void FlightContextUpdateIMUData (FlightContext* pContext, Vec3 accel, Vec3 gyro);
void FlightContextUpdateFlightMode (FlightContext* pContext, uint32_t flightMode);
// void FlightContextUpdateCurrentVelocities(FlightContext *pContext, Vec3 vel, Vec3 angVel);
// void FlightContextUpdateTargetVelocities(FlightContext *pContext, Vec3 vel, Vec3 angVel);


#endif // FLIGHT_CONTEXT_H
