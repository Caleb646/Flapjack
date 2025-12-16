#ifndef DRIVERS_SENSORS_INERTIAL_H
#define DRIVERS_SENSORS_INERTIAL_H

#include <stdbool.h>
#include <stdint.h>

#include "common.h"

#include "cfg/cfg.h"

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

typedef struct AccCfg_s {
    INER_TYPE_t type;
    INER_INTERFACE_ID_t interfaceType;
    BusDeviceCfg_t busCfg;
    eGPIO_ID_t extiGpioId;
    uint8_t alignment;
    uint16_t sampleRateHz;
} AccCfg_t;

typedef struct GyroCfg_s {
    INER_TYPE_t type;
    INER_INTERFACE_ID_t interfaceType;
    BusDeviceCfg_t busCfg;
    eGPIO_ID_t extiGpioId;
    uint8_t alignment;
    uint16_t sampleRateHz;
} GyroCfg_t;

typedef struct MagCfg_s {
    INER_TYPE_t type;
    INER_INTERFACE_ID_t interfaceType;
    BusDeviceCfg_t busCfg;
    eGPIO_ID_t extiGpioId;
    uint8_t alignment;
    uint16_t sampleRateHz;
} MagCfg_t;

CFG_DECLARE (AccCfg_t, AccCfg);
CFG_DECLARE (GyroCfg_t, GyroCfg);
CFG_DECLARE (MagCfg_t, MagCfg);

typedef struct AccDevice_s AccDevice_t;
typedef struct GyroDevice_s GyroDevice_t;
typedef struct MagDevice_s MagDevice_t;

typedef struct InertialDeviceVtable_s {

    union {
        void (*fnAccInit) (AccDevice_t*);
        void (*fnGyroInit) (GyroDevice_t*);
        void (*fnMagInit) (MagDevice_t*);
    };

    union {
        eSTATUS_t (*fnAccReadData) (AccDevice_t*, bool forcePolling, int16_t* pOutData);
        eSTATUS_t (*fnGyroReadData) (GyroDevice_t*, bool forcePolling, int16_t* pOutData);
        eSTATUS_t (*fnMagReadData) (MagDevice_t*, bool forcePolling, int16_t* pOutData);
    };

    union {
        void (*fnAccCalibrate) (AccDevice_t*);
        void (*fnGyroCalibrate) (GyroDevice_t*);
        void (*fnMagCalibrate) (MagDevice_t*);
    };

} InertialDeviceVtable_t;

typedef struct AccDevice_s {
    InertialDeviceVtable_t vtbl;
    BusDevice_t* pBusDevice;
    float scaleFactor;
    uint16_t sampleRateHz;
} AccDevice_t;

typedef struct GyroDevice_s {
    InertialDeviceVtable_t vtbl;
    BusDevice_t* pBusDevice;
    float scaleFactor;
    uint16_t sampleRateHz;
} GyroDevice_t;

typedef struct MagDevice_s {
    InertialDeviceVtable_t vtbl;
    BusDevice_t* pBusDevice;
    float scaleFactor;
    uint16_t sampleRateHz;
} MagDevice_t;

FJ_DECLARE_SHARED (AccDevice_t, e_AccDevice);
FJ_DECLARE_SHARED (GyroDevice_t, e_GyroDevice);
FJ_DECLARE_SHARED (MagDevice_t, e_MagDevice);

eSTATUS_t Init_Inertials (void);
// eSTATUS_t Acc_Update (AccDevice_t* pAccDevice, float dt, bool forcePolling, Vec3f* pOutAcc);
// eSTATUS_t Gyro_Update (GyroDevice_t* pGyroDevice, float dt, bool forcePolling, Vec3f* pOutGyro);
// eSTATUS_t Mag_Update (MagDevice_t* pMagDevice, float dt, bool forcePolling, Vec3f* pOutMag);


#endif // DRIVERS_SENSORS_INERTIAL_H