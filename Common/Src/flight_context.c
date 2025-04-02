#include "flight_context.h"

void FlightContextUpdateAttitude(FlightContext *pContext, Vec3f attitude)
{
    pContext->attitude = attitude;
}

void FlightContextUpdateIMUData(FlightContext *pContext, Vec3 accel, Vec3 gyro)
{
    pContext->imuUnFilteredAccel = accel;
    pContext->imuUnFilteredGyro = gyro;
}
void FlightContextUpdateCurrentVelocities(FlightContext *pContext, Vec3 vel, Vec3 angVel)
{
    pContext->curVel = vel;
    pContext->curAngVel = angVel;
}
void FlightContextUpdateTargetVelocities(FlightContext *pContext, Vec3 vel, Vec3 angVel)
{
    pContext->targetVel = vel;
    pContext->targetAngVel = angVel;
}
void FlightContextUpdateFlightMode(FlightContext *pContext, uint32_t flightMode)
{
    pContext->flightMode = flightMode;
}