#include "flight_context.h"

void FlightContextUpdateCurrentAttitude(FlightContext *pContext, Vec3f attitude)
{
    pContext->currentAttitude = attitude;
}

void FlightContextUpdateTargetAttitude(FlightContext *pContext, Vec3f attitude)
{
    pContext->targetAttitude = attitude;
}

void FlightContextUpdateIMUData(FlightContext *pContext, Vec3 accel, Vec3 gyro)
{
    pContext->imuUnFilteredAccel = accel;
    pContext->imuUnFilteredGyro = gyro;
}

void FlightContextUpdateFlightMode(FlightContext *pContext, uint32_t flightMode)
{
    pContext->flightMode = flightMode;
}
// void FlightContextUpdateCurrentVelocities(FlightContext *pContext, Vec3 vel, Vec3 angVel)
// {
//     pContext->curVel = vel;
//     pContext->curAngVel = angVel;
// }
// void FlightContextUpdateTargetVelocities(FlightContext *pContext, Vec3 vel, Vec3 angVel)
// {
//     pContext->targetVel = vel;
//     pContext->targetAngVel = angVel;
// }
