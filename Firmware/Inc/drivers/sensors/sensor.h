#ifndef DRIVERS_SENSORS_INERTIAL_H
#define DRIVERS_SENSORS_INERTIAL_H

#include <stdbool.h>
#include <stdint.h>

#include "common.h"

#include "cfg/cfg.h"

#include "drivers/driver.h"

#include "drivers/bus/bus_defs.h"

typedef uint8_t SENSOR_TYPE_t;
enum {
    SENSOR_TYPE_NULL = 0,
    SENSOR_TYPE_ACC  = 1,
    SENSOR_TYPE_GYR  = 2,
    SENSOR_TYPE_MAG  = 3,
    SENSOR_TYPE_IMU  = 4,
    SENSOR_TYPE_MARG = 5
};

typedef uint8_t SENSOR_ID_t;
enum { SENSOR_ID_NULL = 0, SENSOR_ID_BMI323, SENSOR_ID_MMC5983 };

typedef struct AccCfg_s {
    SENSOR_TYPE_t type;
    SENSOR_ID_t id;
    BusDeviceCfg_t busCfg;
    eGPIO_ID_t extiGpioId;
    uint8_t alignment;
    uint16_t sampleRateHz;
} AccCfg_t;

typedef struct GyroCfg_s {
    SENSOR_TYPE_t type;
    SENSOR_ID_t id;
    BusDeviceCfg_t busCfg;
    eGPIO_ID_t extiGpioId;
    uint8_t alignment;
    uint16_t sampleRateHz;
} GyroCfg_t;

typedef struct MagCfg_s {
    SENSOR_TYPE_t type;
    SENSOR_ID_t id;
    BusDeviceCfg_t busCfg;
    eGPIO_ID_t extiGpioId;
    uint8_t alignment;
    uint16_t sampleRateHz;
} MagCfg_t;

typedef struct AccDevice_s AccDevice_t;
typedef struct GyroDevice_s GyroDevice_t;
typedef struct MagDevice_s MagDevice_t;

typedef struct AccDevice_s {
    eSTATUS_t (*fnRead) (AccDevice_t*, bool forcePolling, int16_t* pOutData);
    BusDevice_t* pBusDevice;
    float scaleFactor;
    uint16_t sampleRateHz;
} AccDevice_t;

typedef struct Acc_s {
    AccDevice_t dev;
    int16_t rawData[3];
    Vec3f scaledData;
    Vec3f filteredData;
} Acc_t;

typedef struct GyroDevice_s {
    eSTATUS_t (*fnRead) (GyroDevice_t*, bool forcePolling, int16_t* pOutData);
    BusDevice_t* pBusDevice;
    float scaleFactor;
    uint16_t sampleRateHz;
} GyroDevice_t;

typedef struct Gyro_s {
    GyroDevice_t dev;
    int16_t rawData[3];
    Vec3f scaledData;
    Vec3f filteredData;
} Gyro_t;

typedef struct MagDevice_s {
    eSTATUS_t (*fnRead) (MagDevice_t*, bool forcePolling, int16_t* pOutData);
    BusDevice_t* pBusDevice;
    float scaleFactor;
    uint16_t sampleRateHz;
} MagDevice_t;

typedef struct Mag_s {
    MagDevice_t dev;
    int16_t rawData[3];
    Vec3f scaledData;
    Vec3f filteredData;
} Mag_t;

CFG_DECLARE (AccCfg_t, AccCfg);
CFG_DECLARE (GyroCfg_t, GyroCfg);
CFG_DECLARE (MagCfg_t, MagCfg);

FJ_DECLARE_SHARED (Acc_t, e_Acc);
FJ_DECLARE_SHARED (Gyro_t, e_Gyro);
FJ_DECLARE_SHARED (Mag_t, e_Mag);

eSTATUS_t Sensors_Init (void);

static inline Acc_t* Acc_GetMutable (void) {
    return &e_Acc;
}
eSTATUS_t Acc_Update (bool forcePolling);
// eSTATUS_t Acc_Filter (void);
bool Acc_IsAvailable (void);

static inline Gyro_t* Gyro_GetMutable (void) {
    return &e_Gyro;
}
eSTATUS_t Gyro_Update (bool forcePolling);
// eSTATUS_t Gyro_Filter (void);
bool Gyro_IsAvailable (void);

static inline Mag_t* Mag_GetMutable (void) {
    return &e_Mag;
}
eSTATUS_t Mag_Update (bool forcePolling);
// eSTATUS_t Mag_Filter (void);
bool Mag_IsAvailable (void);

#endif // DRIVERS_SENSORS_INERTIAL_H