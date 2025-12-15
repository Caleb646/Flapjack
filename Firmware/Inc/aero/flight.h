#ifndef AERO_FLIGHT_H
#define AERO_FLIGHT_H

#include <stdbool.h>
#include <stdint.h>

#include "common.h"

#include "drivers/sensors/inertial/inertial.h"

AccDevice_t* Flight_GetActive_AccDevice (void);
GyroDevice_t* Flight_GetActive_GyroDevice (void);
MagDevice_t* Flight_GetActive_MagDevice (void);
bool Flight_HasAcc (void);
bool Flight_HasGyro (void);
bool Flight_HasMag (void);
eSTATUS_t Acc_Update (AccDevice_t* pAccDevice, float dt, bool forcePolling, Vec3f* pOutAcc);
eSTATUS_t Gyro_Update (GyroDevice_t* pGyroDevice, float dt, bool forcePolling, Vec3f* pOutGyro);
eSTATUS_t Mag_Update (MagDevice_t* pMagDevice, float dt, bool forcePolling, Vec3f* pOutMag);

eSTATUS_t Init_Motion (void);
eSTATUS_t Motion_Update (Vec3f* const pidAtt, float targetThrottle, float dt);


#endif // AERO_FLIGHT_H