#ifndef DRIVERS_SENSORS_CNS_DEFS_H
#define DRIVERS_SENSORS_CNS_DEFS_H

#include <stdint.h>

#include "common.h"

#include "drivers/bus/bus_defs.h"

typedef uint8_t INERTIAL_DEVICE_ID_t;
enum {
    INERTIAL_DEVICE_ID_NULL = 0,

    INERTIAL_DEVICE_ID_ACC_1 = 1,
    INERTIAL_DEVICE_ID_GYR_1 = 2,
    INERTIAL_DEVICE_ID_MAG_1 = 3,

    INERTIAL_DEVICE_ID_IMU_1 = INERTIAL_DEVICE_ID_ACC_1,

    INERTIAL_DEVICE_ID_MARG_1 = INERTIAL_DEVICE_ID_ACC_1,
};

typedef uint8_t INERTIAL_INTERFACE_ID_t;
enum {
    INERTIAL_INTERFACE_ID_NULL   = 0,
    INERTIAL_INTERFACE_ID_BMI323 = 1,
};

typedef struct ACCDevice_s ACCDevice_t;
typedef struct GYRODevice_s GYRODevice_t;
typedef struct IMUDevice_s IMUDevice_t;
typedef struct MARGDevice_s MARGDevice_t;

typedef struct InertialDeviceVtable_s {

    union {
        void (*fnAccInit) (ACCDevice_t*);
        void (*fnGyroInit) (GYRODevice_t*);
        void (*fnIMUInit) (IMUDevice_t*);
        void (*fnMARGInit) (MARGDevice_t*);
    };

    union {
        void (*fnAccReadData) (ACCDevice_t*, bool forcePolling, int16_t* pOutData);
        void (*fnGyroReadData) (GYRODevice_t*, bool forcePolling, int16_t* pOutData);
        void (*fnIMUReadData) (IMUDevice_t*, bool forcePolling, int16_t* pOutData);
        void (*fnMARGReadData) (MARGDevice_t*, bool forcePolling, int16_t* pOutData);
    };

    union {
        void (*fnAccCalibrate) (ACCDevice_t*);
        void (*fnGyroCalibrate) (GYRODevice_t*);
        void (*fnIMUCalibrate) (IMUDevice_t*);
        void (*fnMARGCalibrate) (MARGDevice_t*);
    };

} InertialDeviceVtable_t;

typedef struct InertialDevice_s {
    INERTIAL_DEVICE_ID_t inertialId;
    INERTIAL_INTERFACE_ID_t interfaceId;
    BusDevice_t busDevice;
    InertialDeviceVtable_t vtbl;
} InertialDevice_t;

typedef uint8_t CNS_ID_t;
enum {
    CNS_ID_NULL    = 0,
    CNS_ID_PRIMARY = 1,
};


#endif // DRIVERS_SENSORS_CNS_DEFS_H