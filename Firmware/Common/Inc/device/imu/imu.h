#ifndef SENSORS_IMU_H
#define SENSORS_IMU_H

#include "core/core.h"
#include "device/imu/bmixxx.h"
#include "hal.h"
#include "target.h"

#include "drivers/bus/spi.h"

#include <stdint.h>
#include <string.h>


#define IMU_LOG_CALIB_DATA(rslt, error) \
    LOG_DATA (LOG_DATA_TYPE_IMU_CALIB, "{\"rslt\":%u,\"error\":%u}", rslt, error)

typedef uint8_t IMU_ACC_RANGE;
enum {
    eIMU_ACC_RANGE_2G  = 0x00,
    eIMU_ACC_RANGE_4G  = 0x01,
    eIMU_ACC_RANGE_8G  = 0x02,
    eIMU_ACC_RANGE_16G = 0x03
};

// Accelerometer output data rate in Hertz
typedef uint8_t IMU_ACC_ODR;
enum {
    eIMU_ACC_ODR_50   = 0x07,
    eIMU_ACC_ODR_100  = 0x08,
    eIMU_ACC_ODR_200  = 0x09,
    eIMU_ACC_ODR_400  = 0x0A,
    eIMU_ACC_ODR_800  = 0x0B,
    eIMU_ACC_ODR_1600 = 0x0C
};

typedef uint8_t IMU_ACC_BW;
enum { eIMU_ACC_BW_HALF = BMI3_ACC_BW_ODR_HALF, eIMU_ACC_BW_QUARTER = BMI3_ACC_BW_ODR_QUARTER };

typedef uint8_t IMU_ACC_AVG;
enum {
    eIMU_ACC_AVG_1  = BMI3_ACC_AVG1,
    eIMU_ACC_AVG_2  = BMI3_ACC_AVG2,
    eIMU_ACC_AVG_4  = BMI3_ACC_AVG4,
    eIMU_ACC_AVG_8  = BMI3_ACC_AVG8,
    eIMU_ACC_AVG_16 = BMI3_ACC_AVG16,
    eIMU_ACC_AVG_32 = BMI3_ACC_AVG32,
    eIMU_ACC_AVG_64 = BMI3_ACC_AVG64
};

typedef uint8_t IMU_ACC_MODE;
enum {
    eIMU_ACC_MODE_DISABLE   = BMI3_ACC_MODE_DISABLE,
    eIMU_ACC_MODE_LOW_PWR   = BMI3_ACC_MODE_LOW_PWR,
    eIMU_ACC_MODE_NORMAL    = BMI3_ACC_MODE_NORMAL,
    eIMU_ACC_MODE_HIGH_PERF = BMI3_ACC_MODE_HIGH_PERF
};

// degrees per second
typedef uint8_t IMU_GYRO_RANGE;
enum {
    eIMU_GYRO_RANGE_125  = 0x00,
    eIMU_GYRO_RANGE_250  = 0x01,
    eIMU_GYRO_RANGE_500  = 0x02,
    eIMU_GYRO_RANGE_1000 = 0x03,
    eIMU_GYRO_RANGE_2000 = 0x04
};

// Gyro output data rate in Hertz
typedef uint8_t IMU_GYRO_ODR;
enum {
    eIMU_GYRO_ODR_50   = 0x07,
    eIMU_GYRO_ODR_100  = 0x08,
    eIMU_GYRO_ODR_200  = 0x09,
    eIMU_GYRO_ODR_400  = 0x0A,
    eIMU_GYRO_ODR_800  = 0x0B,
    eIMU_GYRO_ODR_1600 = 0x0C
};

typedef uint8_t IMU_GYRO_BW;
enum { eIMU_GYRO_BW_HALF = BMI3_GYR_BW_ODR_HALF, eIMU_GYRO_BW_QUARTER = BMI3_GYR_BW_ODR_HALF };

typedef uint8_t IMU_GYRO_AVG;
enum {
    eIMU_GYRO_AVG_1  = BMI3_GYR_AVG1,
    eIMU_GYRO_AVG_2  = BMI3_GYR_AVG2,
    eIMU_GYRO_AVG_4  = BMI3_GYR_AVG4,
    eIMU_GYRO_AVG_8  = BMI3_GYR_AVG8,
    eIMU_GYRO_AVG_16 = BMI3_GYR_AVG16,
    eIMU_GYRO_AVG_32 = BMI3_GYR_AVG32,
    eIMU_GYRO_AVG_64 = BMI3_GYR_AVG64
};

typedef uint8_t IMU_GYRO_MODE;
enum {
    eIMU_GYRO_MODE_DISABLE   = BMI3_GYR_MODE_DISABLE,
    eIMU_GYRO_MODE_SUSPEND   = BMI3_GYR_MODE_SUSPEND,
    eIMU_GYRO_MODE_LOW_PWR   = BMI3_GYR_MODE_LOW_PWR,
    eIMU_GYRO_MODE_NORM      = BMI3_GYR_MODE_NORMAL,
    eIMU_GYRO_MODE_HIGH_PERF = BMI3_GYR_MODE_HIGH_PERF
};

typedef eSTATUS_t eIMU_STATUS;
enum {
    eIMU_COM_FAILURE        = -1,
    eIMU_RW_BUFFER_OVERFLOW = -2,
    eIMU_NULL_PTR           = -3,
    eIMU_HARDWARE_ERR       = -4,
};

typedef struct {
    uint16_t err;
    /* Indicates fatal error */
    uint8_t fatalErr;
    /* Overload of the feature engine detected. */
    uint8_t featEngOvrld;
    /* Watchdog timer of the feature engine triggered. */
    uint8_t featEngWd;
    /* Indicates accel configuration error */
    uint8_t accConfErr;
    /* Indicates gyro configuration error */
    uint8_t gyrConfErr;
    /* Indicates SDR parity error */
    uint8_t i3cErr0;
    /* Indicates I3C error */
    uint8_t i3cErr1;
} IMUErr;

typedef struct {
    union {
        uint16_t raw;
        struct {
            uint16_t por_detected : 1;
            uint16_t reserved_1 : 4;
            uint16_t drdyTemp : 1;
            uint16_t drdyGyro : 1;
            uint16_t drdyAccel : 1;
        };
    };
} IMU_SysStatusReg_t;

typedef struct {
    union {
        uint16_t raw;
        struct {
            uint16_t unused : 10;
            uint16_t errStatus : 1;
            uint16_t drdyTemp : 1;
            uint16_t drdyGyro : 1;
            uint16_t drdyAccel : 1;
            uint16_t fifoWatermark : 1;
            uint16_t fifoFull : 1;
        } int_1;
    };
} IMU_INTStatusReg_t;

typedef struct {
    union {
        uint16_t raw;
        struct {
            uint16_t error_status : 4;
            uint16_t sc_st_complete : 1;
            uint16_t gyro_sc_complete : 1;
            uint16_t st_result : 1;
            uint16_t sample_rate_err : 1;
            uint16_t reserved_1 : 2;
            uint16_t axis_map_complete : 1;
            uint16_t state : 2;
            uint16_t reserved_2 : 3;
        } feat_1;
    };
} IMU_FeatureReg_t;

typedef struct {
    IMU_ACC_RANGE range;
    IMU_ACC_ODR odr;
    IMU_ACC_BW bw;
    IMU_ACC_AVG avg;
    IMU_ACC_MODE mode;
} IMUAccConf;

typedef struct {
    IMU_GYRO_RANGE range;
    IMU_GYRO_ODR odr;
    IMU_GYRO_BW bw;
    IMU_GYRO_AVG avg;
    IMU_GYRO_MODE mode;
} IMUGyroConf;

/*
 *   0x0 --> x=x; y=y; z=z;
 *   0x1 --> x=y; y=x; z=z;
 *   0x2 --> x=x; y=z; z=y;
 *   0x3 --> x=z; y=x; z=y;
 *   0x4 --> x=y; y=z; z=x;
 *   0x5 --> x=z; y=y; z=x;
 */
typedef uint8_t eIMU_AXES_REMAP_t;
enum {
    eIMU_AXES_REMAP_XYZ = 0x00,
    eIMU_AXES_REMAP_YXZ = 0x01,
    eIMU_AXES_REMAP_ZXY = 0x02,
    eIMU_AXES_REMAP_XZY = 0x03,
    eIMU_AXES_REMAP_YZX = 0x04,
    eIMU_AXES_REMAP_ZYX = 0x05
};

typedef uint8_t eIMU_AXES_DIR_t;
enum { eIMU_AXES_DIR_DEFAULT = 0x00, eIMU_AXES_DIR_INVERTED = 0x01 };

typedef struct {
    eIMU_AXES_REMAP_t remap;
    eIMU_AXES_DIR_t xDir;
    eIMU_AXES_DIR_t yDir;
    eIMU_AXES_DIR_t zDir;
} IMUAxesRemapConf;

typedef struct {
    IMUAccConf aconf;
    IMUGyroConf gconf;
    IMUAxesRemapConf axesRemapConf;
} IMUInitConf_t;

typedef struct {
    eDEVICE_ID_t deviceId;
    // eBUS_ID_t busId;
    SpiDev_t spiDev;

    IMUAccConf aconf;
    IMUGyroConf gconf;
    eSTATUS_t status;
    uint32_t msLastAccUpdateTime;
    uint32_t msLastGyroUpdateTime;
    // BusVTable_t bus;
    uint8_t nBusDummyBytes;
    bool usingEXTIInterrupt;
    bool isInitialized;
    bool volatile gyroDataUpdated;
    bool volatile accelDataUpdated;

    Vec3i rawAccel;
    Vec3f accelData;
    Vec3f accelFilteredData;

    Vec3i rawGyro;
    Vec3f gyroData;
    Vec3f gyroFilteredData;
} IMU_t;

// typedef IMU_t volatile vIMU_t;
typedef IMU_t vIMU_t;

FJ_DECLARE_SHARED (IMU_t, g_IMU);

#ifdef UNIT_TEST

eSTATUS_t IMUSendCmd (vIMU_t* pIMU, uint16_t cmd);
eSTATUS_t IMUGetFeatureStatus (vIMU_t* pIMU, uint16_t featureRegAddr, IMU_FeatureReg_t* pOutStatus);
eSTATUS_t IMUGetINTStatus (vIMU_t* pIMU, IMU_INTStatusReg_t*);
eSTATUS_t IMUGetSysStatus (vIMU_t* pIMU, IMU_SysStatusReg_t* pOutStatus);
eSTATUS_t IMUReadReg (vIMU_t* pIMU, uint8_t reg, uint8_t* pBuf, uint32_t len);
eSTATUS_t IMUWriteReg (vIMU_t* pIMU, uint8_t reg, uint8_t* pBuf, uint32_t len);
eSTATUS_t IMUUpdateRawGyro (vIMU_t* pIMU);
eSTATUS_t IMUUpdateRawAccel (vIMU_t* pIMU);
eSTATUS_t IMUSetAxesRemap (vIMU_t* pIMU, IMUAxesRemapConf remap);
eSTATUS_t IMUSoftReset (vIMU_t* pIMU);
eSTATUS_t IMUGetConf_ (vIMU_t* pIMU, IMUAccConf* pAConf, IMUGyroConf* pGConf, uint8_t altConfFlag);
eSTATUS_t IMUSetConf_ (vIMU_t* pIMU, IMUAccConf const* pAConf, IMUGyroConf const* pGConf, uint8_t altConfFlag);
eSTATUS_t IMUCalibrate (vIMU_t* pIMU, uint8_t calibSelection, uint8_t applyCorrection);
eSTATUS_t IMUSetupInterrupts (vIMU_t* pIMU);
eSTATUS_t IMUEnableInterrupts (vIMU_t* pIMU);
eSTATUS_t IMUDisableInterrupts (vIMU_t* pIMU);
eSTATUS_t
IMUConvertRaw (IMU_ACC_RANGE aRange, Vec3i ra, IMU_GYRO_RANGE gRange, Vec3i rg, Vec3f* pAccelOut, Vec3f* pGyroOut);

#endif

eSTATUS_t IMU_Init_ (IMUInitConf_t conf, vIMU_t* pOutIMU);
static inline eSTATUS_t IMU_Init (void) {
    IMUInitConf_t conf = { 0 };

    IMUAccConf aconf = { 0 };
    aconf.odr        = eIMU_ACC_ODR_400;
    aconf.range      = eIMU_ACC_RANGE_2G;
    aconf.avg        = eIMU_ACC_AVG_16;
    aconf.bw         = eIMU_ACC_BW_HALF;
    aconf.mode       = eIMU_ACC_MODE_HIGH_PERF;

    IMUGyroConf gconf = { 0 };
    gconf.odr         = eIMU_GYRO_ODR_400;
    gconf.range       = eIMU_GYRO_RANGE_250;
    gconf.avg         = eIMU_GYRO_AVG_16;
    gconf.bw          = eIMU_GYRO_BW_HALF;
    gconf.mode        = eIMU_GYRO_MODE_HIGH_PERF;

    IMUAxesRemapConf axesRemap = { 0 };
    axesRemap.remap            = eIMU_AXES_REMAP_YXZ;
    axesRemap.xDir             = eIMU_AXES_DIR_INVERTED;
    axesRemap.yDir             = eIMU_AXES_DIR_INVERTED;
    axesRemap.zDir             = eIMU_AXES_DIR_INVERTED;

    conf.aconf         = aconf;
    conf.gconf         = gconf;
    conf.axesRemapConf = axesRemap;
    return IMU_Init_ (conf, &g_IMU);
}
eSTATUS_t IMUHandleErr (vIMU_t* pIMU);
eSTATUS_t IMU_Update (vIMU_t* pIMU, bool forcePolling, Vec3f* pOutputAccel, Vec3f* pOutputGyro);
eSTATUS_t IMUGetConf (vIMU_t* pIMU, IMUAccConf* pAConf, IMUGyroConf* pGConf);
eSTATUS_t IMUGetAltConf (vIMU_t* pIMU, IMUAccConf* pAConf, IMUGyroConf* pGConf);
eSTATUS_t IMUSetConf (vIMU_t* pIMU, IMUAccConf const* pAConf, IMUGyroConf const* pGConf);
eSTATUS_t IMUSetAltConf (vIMU_t* pIMU, IMUAccConf const* pAConf, IMUGyroConf const* pGConf);
eSTATUS_t IMUCompareConfs (IMUAccConf aconf, IMUGyroConf gconf, IMUAccConf aconf2, IMUGyroConf gconf2);
vIMU_t const* IMUGetActiveDevice (void);
vIMU_t* IMU_GetMutableActiveDevice (void);
void IMU2CPUInterruptHandler (vIMU_t* pIMU);


#endif // SENSORS_IMU_H
