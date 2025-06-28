#include "flight_context.h"
#include <string.h>

void FlightContextUpdateCurrentAttitude (FlightContext* pContext, Vec3f attitude) {
    pContext->currentAttitude = attitude;
}

void FlightContextUpdateTargetAttitudeThrottle (FlightContext* pContext, Vec3f attitude, float throttle) {
    pContext->targetAttitude = attitude;
    pContext->targetThrottle = throttle;
}

void FlightContextUpdatePIDAttitude (FlightContext* pContext, Vec3f attitude) {
    pContext->pidAttitude = attitude;
}

/*
 * \brief IMU collects it accel and gyro data in millimeters and
 * millidegrees. This is converted to meters and degrees
 */
void FlightContextUpdateIMUData (FlightContext* pContext, Vec3 accel, Vec3 gyro) {
    pContext->imuUnFilteredAccel.x = ((float)accel.x) / 1000.0F;
    pContext->imuUnFilteredAccel.y = ((float)accel.y) / 1000.0F;
    pContext->imuUnFilteredAccel.z = ((float)accel.z) / 1000.0F;

    pContext->imuUnFilteredGyro.x = ((float)gyro.x) / 1000.0F;
    pContext->imuUnFilteredGyro.y = ((float)gyro.y) / 1000.0F;
    pContext->imuUnFilteredGyro.z = ((float)gyro.z) / 1000.0F;
}

void FlightContextUpdateFlightMode (FlightContext* pContext, uint32_t flightMode) {
    pContext->flightMode = flightMode;
}

STATUS_TYPE FlightContextInit (FlightContext* pContext) {
    memset (pContext, 0, sizeof (FlightContext));
    return eSTATUS_SUCCESS;
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
