#ifndef AERO_FLIGHT_H
#define AERO_FLIGHT_H

#include <stdbool.h>
#include <stdint.h>

#include "common.h"

#include "drivers/sensors/inertial/inertial.h"

ACCDevice_t* Flight_GetActive_AccDevice (void);
GYRODevice_t* Flight_GetActive_GyroDevice (void);
MAGDevice_t* Flight_GetActive_MagDevice (void);
bool Flight_HasAcc (void);
bool Flight_HasGyro (void);
bool Flight_HasMag (void);
eSTATUS_t Acc_Update (ACCDevice_t* pAccDevice, float dt, bool forcePolling, Vec3f* pOutAcc);
eSTATUS_t Gyro_Update (GYRODevice_t* pGyroDevice, float dt, bool forcePolling, Vec3f* pOutGyro);
eSTATUS_t Mag_Update (MAGDevice_t* pMagDevice, float dt, bool forcePolling, Vec3f* pOutMag);

eSTATUS_t Init_Motion (void);
eSTATUS_t Motion_Update (Vec3f* const pidAtt, float targetThrottle, float dt);


#endif // AERO_FLIGHT_H