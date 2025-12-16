#ifndef AERO_FLIGHT_H
#define AERO_FLIGHT_H

#include <stdbool.h>
#include <stdint.h>

#include "common.h"

#include "drivers/sensors/inertial/inertial.h"

typedef struct Acc_s {
    AccDevice_t dev;
    int16_t rawData[3];
    Vec3f scaledData;
    Vec3f filteredData;
} Acc_t;

FJ_DECLARE_SHARED (Acc_t, e_Acc);

eSTATUS_t Acc_Update (float dt, bool forcePolling);
eSTATUS_t Gyro_Update (float dt, bool forcePolling);
eSTATUS_t Mag_Update (float dt, bool forcePolling);


#endif // AERO_FLIGHT_H