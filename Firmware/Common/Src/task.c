#include <stdio.h>

#include "flight.h"
#include "hal.h"
#include "task.h"

#include "core/core.h"

#include "target.h"

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

    FlightData_t* pFlightData = Fc_Get ();
    vIMU_t* pIMU              = Imu_Get ();
    vFilter_t* pFilter        = Filter_GetMutableActiveFilter ();
    float dt                  = ((float)usCurrentTime - (float)usDeltaTime) / 1000000.0F;

    Vec3f outputAttitude = { 0.0F };
    // TODO: add option for using magnetometer data in the filter
    eSTATUS_t status = Filter_Update (pFilter, &pIMU->accelData, &pIMU->gyroData, NULL, dt, &outputAttitude);
    if (STATUS_FAIL (status)) {
        LOG_ERROR ("Failed to update filter");
        return eSTATUS_FAILURE;
    }
    pFlightData->current[AXIS_IDX_ROLL]  = outputAttitude.roll;
    pFlightData->current[AXIS_IDX_PITCH] = outputAttitude.pitch;
    pFlightData->current[AXIS_IDX_YAW]   = outputAttitude.yaw;

    return status;
}

eSTATUS_t TaskIMUUpdate (uint32_t usCurrentTime, uint32_t usDeltaTime) {

    vIMU_t* pIMUDev = Imu_Get ();
    return IMU_Update (pIMUDev, false, &pIMUDev->accelData, &pIMUDev->gyroData);
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