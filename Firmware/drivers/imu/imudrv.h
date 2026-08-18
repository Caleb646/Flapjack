#ifndef DRIVERS_IMU_IMU_DRIVER_H
#define DRIVERS_IMU_IMU_DRIVER_H

/*
 * Generic, device-agnostic IMU driver interface.
 *
 * The application layer (sensors/imu.c) talks to the hardware only through the
 * ImuDriver_t vtable below, so a BMI323 backend and a simulation backend are
 * interchangeable at runtime. A backend implements `init`/`read` and exposes a
 * `*_GetDriver()` accessor returning an ImuDriver_t bound to its private state.
 *
 * `read` returns samples already converted to engineering units (accel m/s2,
 * gyro deg/s) in the configured orientation - range-dependent scaling and axis
 * remap are backend concerns.
 *
 * This header is the boundary: it must not depend on any backend header
 * (bmi323.h / bmixxx.h).
 */

#include "core/core.h"

#include <stdbool.h>
#include <stdint.h>

// Accelerometer full-scale range (g).
typedef uint8_t eImuAccRange_t;
enum {
    eIMU_ACC_RANGE_2G  = 0x00,
    eIMU_ACC_RANGE_4G  = 0x01,
    eIMU_ACC_RANGE_8G  = 0x02,
    eIMU_ACC_RANGE_16G = 0x03
};

// Gyroscope full-scale range (deg/s).
typedef uint8_t eImuGyroRange_t;
enum {
    eIMU_GYRO_RANGE_125  = 0x00,
    eIMU_GYRO_RANGE_250  = 0x01,
    eIMU_GYRO_RANGE_500  = 0x02,
    eIMU_GYRO_RANGE_1000 = 0x03,
    eIMU_GYRO_RANGE_2000 = 0x04
};

// Output data rate (Hz), shared by accel and gyro.
typedef uint8_t eImuOdr_t;
enum {
    eIMU_ODR_50   = 0x07,
    eIMU_ODR_100  = 0x08,
    eIMU_ODR_200  = 0x09,
    eIMU_ODR_400  = 0x0A,
    eIMU_ODR_800  = 0x0B,
    eIMU_ODR_1600 = 0x0C
};

/*
 *   XYZ --> x=x; y=y; z=z;
 *   YXZ --> x=y; y=x; z=z;
 *   ZXY --> x=x; y=z; z=y;
 *   XZY --> x=z; y=x; z=y;
 *   YZX --> x=y; y=z; z=x;
 *   ZYX --> x=z; y=y; z=x;
 */
typedef uint8_t eImuAxesRemap_t;
enum {
    eIMU_AXES_REMAP_XYZ = 0x00,
    eIMU_AXES_REMAP_YXZ = 0x01,
    eIMU_AXES_REMAP_ZXY = 0x02,
    eIMU_AXES_REMAP_XZY = 0x03,
    eIMU_AXES_REMAP_YZX = 0x04,
    eIMU_AXES_REMAP_ZYX = 0x05
};

typedef uint8_t eImuAxesDir_t;
enum { eIMU_AXES_DIR_DEFAULT = 0x00, eIMU_AXES_DIR_INVERTED = 0x01 };

typedef struct {
    eImuAxesRemap_t remap;
    eImuAxesDir_t xDir;
    eImuAxesDir_t yDir;
    eImuAxesDir_t zDir;
} ImuAxesRemap_t;

typedef struct {
    ImuAxesRemap_t orientation;
    eImuAccRange_t accRange;
    eImuGyroRange_t gyroRange;
    eImuOdr_t odr;
} ImuDriverConf_t;

typedef struct ImuDriver_s {
    void* ctx;
    eSTATUS_t (*Read) (void* ctx, bool forcePolling, Vec3f* pAccel, Vec3f* pGyro);
    bool (*IsDataReady) (void* ctx);
} ImuDriver_t;

eSTATUS_t ImuDrv_Init(ImuDriverConf_t const* pConf, ImuDriver_t* pOutDriver);

#endif // DRIVERS_IMU_IMU_DRIVER_H
