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
 * `read` returns samples converted to engineering units (accel m/s2, gyro
 * deg/s) in the SENSOR's own frame and sign convention - a backend applies only
 * what the part itself dictates, i.e. range-dependent scaling. Mounting
 * rotation and this project's accelerometer sign convention are NOT backend
 * concerns: devices/imu.c owns both, so every backend reports the same thing
 * and the conversion is written once (common/align.h).
 *
 * This header is the boundary: it must not depend on any backend header
 * (bmi323.h / bmixxx.h).
 */

#include "core/core.h"

#include "drivers/device.h"

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

typedef struct {
    Vec3f accel; // m/s^2
    Vec3f gyro;  // deg/s
} ImuData_t;

typedef struct ImuDriver_s {
    void* ctx;
    eSTATUS_t (*Read) (void* ctx, bool forcePolling, ImuData_t* pOutData);
    bool (*IsDataReady) (void* ctx);
    struct {
        eImuAccRange_t accRange;
        eImuGyroRange_t gyroRange;
        eImuOdr_t odr;
        /* Optional. With a Notify set the backend routes the part's data-ready
         * output to its INT pin and wires the EXTI line; leave it zeroed to
         * poll. Kept here rather than consumed at init so a caller can still
         * see whether the interrupt path was asked for. */
        DataReadySignal_t signal;
    } cfg;
} ImuDriver_t;

/*
 * Fill pOutDriver->cfg first; this reads it and does NOT clear the struct, the
 * same contract UartPort_Init and SpiDev_Init keep.
 */
eSTATUS_t ImuDrv_Init (ImuDriver_t* pOutDriver);

#endif // DRIVERS_IMU_IMU_DRIVER_H
