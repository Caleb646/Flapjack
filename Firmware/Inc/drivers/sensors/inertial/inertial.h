#ifndef DRIVERS_SENSORS_INERTIAL_H
#define DRIVERS_SENSORS_INERTIAL_H

#include <stdbool.h>
#include <stdint.h>

#include "common.h"

#include "drivers/driver.h"

#include "drivers/bus/bus_defs.h"

typedef uint8_t INER_TYPE_t;
enum {
    INER_TYPE_NULL = 0,
    INER_TYPE_ACC  = 1,
    INER_TYPE_GYR  = 2,
    INER_TYPE_MAG  = 3,
    INER_TYPE_IMU  = 4,
    INER_TYPE_MARG = 5
};

typedef uint8_t INER_INTERFACE_ID_t;
enum { INER_INTERFACE_ID_NULL = 0, INER_INTERFACE_ID_BMI323, INER_INTERFACE_ID_MMC5983 };

typedef struct ACCDevice_s ACCDevice_t;
typedef struct GYRODevice_s GYRODevice_t;
typedef struct IMUDevice_s IMUDevice_t;
typedef struct MARGDevice_s MARGDevice_t;

typedef struct InertialDeviceVtable_s {

    union {
        void (*fnAccInit) (ACCDevice_t*);
        void (*fnGyroInit) (GYRODevice_t*);
        void (*fnMagInit) (IMUDevice_t*);
    };

    union {
        void (*fnAccReadData) (ACCDevice_t*, bool forcePolling, int16_t* pOutData);
        void (*fnGyroReadData) (GYRODevice_t*, bool forcePolling, int16_t* pOutData);
        void (*fnMagReadData) (IMUDevice_t*, bool forcePolling, int16_t* pOutData);
    };

    union {
        void (*fnAccCalibrate) (ACCDevice_t*);
        void (*fnGyroCalibrate) (GYRODevice_t*);
        void (*fnMagCalibrate) (IMUDevice_t*);
    };

} InertialDeviceVtable_t;

typedef struct ACCDevice_s {
    INER_TYPE_t type;
    INER_INTERFACE_ID_t interfaceId;
    BusDevice_t* pBusDevice;
    float scaleFactor;
    uint16_t sampleRateHz;
} ACCDevice_t;

typedef struct GYRODevice_s {
    INER_TYPE_t type;
    INER_INTERFACE_ID_t interfaceId;
    BusDevice_t* pBusDevice;
    float scaleFactor;
    uint16_t sampleRateHz;
} GYRODevice_t;

typedef struct MAGDevice_s {
    INER_TYPE_t type;
    INER_INTERFACE_ID_t interfaceId;
    BusDevice_t* pBusDevice;
    float scaleFactor;
    uint16_t sampleRateHz;
} MAGDevice_t;

DRIVER_DECLARE (ACCDevice_t, AccDevice);
DRIVER_DECLARE (GYRODevice_t, GyroDevice);
DRIVER_DECLARE (MAGDevice_t, MagDevice);

eSTATUS_t Inertials_Init (void);
eSTATUS_t Acc_Update (float dt, bool forcePolling, Vec3f* pOutAcc);
eSTATUS_t Gyro_Update (float dt, bool forcePolling, Vec3f* pOutGyro);
eSTATUS_t Mag_Update (float dt, bool forcePolling, Vec3f* pOutMag);


#endif // DRIVERS_SENSORS_INERTIAL_H