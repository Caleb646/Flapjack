#ifndef DRIVERS_SENSORS_CNS_DEFS_H
#define DRIVERS_SENSORS_CNS_DEFS_H

#include <stdint.h>

#include "common.h"

typedef uint8_t NAV_SENSOR_ID_t;
enum {
    NAV_SENSOR_ID_NULL = 0,

    NAV_SENSOR_ID_ACC_1 = 1,
    NAV_SENSOR_ID_GYR_1 = 2,
    NAV_SENSOR_ID_MAG_1 = 3,

    NAV_SENSOR_ID_IMU_1 = NAV_SENSOR_ID_ACC_1,

    NAV_SENSOR_ID_MARG_1 = NAV_SENSOR_ID_ACC_1,

};

typedef struct ACCDevice_s ACCDevice_t;
typedef struct GYRODevice_s GYRODevice_t;
typedef struct IMUDevice_s IMUDevice_t;
typedef struct MARGDevice_s MARGDevice_t;

typedef struct NavSensorVtable_s {

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

} NavSensorVtable_t;


#endif // DRIVERS_SENSORS_CNS_DEFS_H