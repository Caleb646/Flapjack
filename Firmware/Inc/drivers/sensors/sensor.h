#ifndef DRIVERS_SENSORS_INERTIAL_H
#define DRIVERS_SENSORS_INERTIAL_H

#include <stdbool.h>
#include <stdint.h>

#include "common.h"

#include "cfg/cfg.h"

#include "drivers/bus/bus_defs.h"


typedef uint8_t eSENSOR_IFACE_t;
#define eSENSOR_IFACE_NONE                   0U
#define eSENSOR_IFACE_BMI323                 1U
#define eSENSOR_IFACE_MMC5983                2U

// SENSOR_IFACE_MAKE(BMI323) -> eSENSOR_IFACE_BMI323
#define SENSOR_IFACE_MAKE_EXPAND(IFACE_NAME) eSENSOR_IFACE_##IFACE_NAME
#define SENSOR_IFACE_MAKE(IFACE_NAME)        SENSOR_IFACE_MAKE_EXPAND (IFACE_NAME)

typedef struct AccCfg_s {
    eSENSOR_IFACE_t iface;
    BusDeviceCfg_t busCfg;
    eGPIO_ID_t extiGpioId;
    uint8_t alignment;
    uint16_t sampleRateHz;
} AccCfg_t;

typedef struct GyroCfg_s {
    eSENSOR_IFACE_t iface;
    BusDeviceCfg_t busCfg;
    eGPIO_ID_t extiGpioId;
    uint8_t alignment;
    uint16_t sampleRateHz;
} GyroCfg_t;

typedef struct MagCfg_s {
    eSENSOR_IFACE_t iface;
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

// CFG_DECLARE (AccCfg_t, AccCfg);
// CFG_DECLARE (GyroCfg_t, GyroCfg);
// CFG_DECLARE (MagCfg_t, MagCfg);

FJ_DECLARE_SHARED (Acc_t, e_Acc);
FJ_DECLARE_SHARED (Gyro_t, e_Gyro);
FJ_DECLARE_SHARED (Mag_t, e_Mag);

static inline Acc_t* Acc_GetMutable (void) {
    return &e_Acc;
}
eSTATUS_t Acc_Init (void);
eSTATUS_t Acc_Update (uint32_t currentTimeUs, bool forcePolling);
bool Acc_IsAvailable (void);


static inline Gyro_t* Gyro_GetMutable (void) {
    return &e_Gyro;
}
eSTATUS_t Gyro_Init (void);
eSTATUS_t Gyro_Update (uint32_t currentTimeUs, bool forcePolling);
bool Gyro_IsAvailable (void);

static inline Mag_t* Mag_GetMutable (void) {
    return &e_Mag;
}
eSTATUS_t Mag_Init (void);
eSTATUS_t Mag_Update (uint32_t currentTimeUs, bool forcePolling);
bool Mag_IsAvailable (void);


eSTATUS_t Baro_Init (void);
eSTATUS_t Gps_Init (void);

#endif // DRIVERS_SENSORS_INERTIAL_H