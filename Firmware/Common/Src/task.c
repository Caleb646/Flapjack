#include <stdio.h>

#include "flight.h"
#include "hal.h"
#include "target.h"
#include "task.h"

#include "core/core.h"

#include "device/imu/imu.h"

#include "drivers/sensors/mag/mag.h"

#include "drivers/rx/rx.h"

#include "fc/rc.h"

#include "mc/filter.h"
#include "mc/mixer.h"
#include "mc/pid.h"

#include "drivers/dma.h"
#include "drivers/io/gpio.h"

eSTATUS_t TaskMixerUpdate (uint32_t usCurrentTime, uint32_t usDeltaTime) {

    if (STATUS_FAIL (Mixer_Mix (usCurrentTime))) {
        LOG_ERROR ("Failed to mix PID output to actuator outputs");
        return eSTATUS_FAILURE;
    }
    return Mixer_Update (usCurrentTime);
}

eSTATUS_t TaskPIDUpdate (uint32_t usCurrentTime, uint32_t usDeltaTime) {

    return Pid_Update (usCurrentTime, usDeltaTime);
}

eSTATUS_t TaskAttitudeUpdate (uint32_t usCurrentTime, uint32_t usDeltaTime) {

    eSTATUS_t status          = eSTATUS_SUCCESS;
    Flight_t* pFlight         = Fc_Get ();
    IMU_t* pIMU               = Imu_Get ();
    Mag_t* pMag               = Mag_Get ();
    float dt                  = (float)usDeltaTime / 1000000.0F;

    if (!pIMU && !pMag) {
        LOG_ERROR ("No sensors available for attitude update");
        return eSTATUS_FAILURE;
    }

    Vec3f outputAttitude = { 0.0F };
    if (!pMag) {
        status =
        MadgwickFilter_Update (&pFlight->attitudeFilter, &pIMU->accelData, &pIMU->gyroData, NULL, dt, &outputAttitude);
    } else {
        status =
        MadgwickFilter_Update (&pFlight->attitudeFilter, &pIMU->accelData, &pIMU->gyroData, &pMag->normedData, dt, &outputAttitude);
    }
    pFlight->current[AXIS_IDX_ROLL]  = outputAttitude.roll;
    pFlight->current[AXIS_IDX_PITCH] = outputAttitude.pitch;
    pFlight->current[AXIS_IDX_YAW]   = outputAttitude.yaw;
    return status;
}

eSTATUS_t TaskImu_Update (uint32_t usCurrentTime, uint32_t usDeltaTime) {

    vIMU_t* pIMUDev = Imu_Get ();
    return IMU_Update (pIMUDev, false, &pIMUDev->accelData, &pIMUDev->gyroData);
}

eSTATUS_t TaskMag_Update (uint32_t usCurrentTime, uint32_t usDeltaTime) {

    return Mag_Update (false);
}

eSTATUS_t TaskInterCoreSync (uint32_t usCurrentTime, uint32_t usDeltaTime) {

    return SyncProcessTasks ();
}

eSTATUS_t Task_LogHeartBeat (uint32_t usCurrentTime, uint32_t usDeltaTime) {

    LOG_INFO ("Heartbeat");
    return eSTATUS_SUCCESS;
}

eSTATUS_t Task_LogFlightData (uint32_t usCurrentTime, uint32_t usDeltaTime) {

    Imu_LogData ();
    Fc_LogData ();
    Pid_LogData ();

    return eSTATUS_SUCCESS;
}

eSTATUS_t Task_RxUpdate (uint32_t usCurrentTime, uint32_t usDeltaTime) {

    return Rx_Update (usCurrentTime, usDeltaTime);
}

eSTATUS_t Task_RcUpdate (uint32_t usCurrentTime, uint32_t usDeltaTime) {

    return Rc_Update (usCurrentTime, usDeltaTime);
}