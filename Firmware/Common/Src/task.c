#include <stdio.h>

#include "control.h"
#include "fcstate.h"
#include "flight.h"
#include "hal.h"
#include "task.h"

#include "core/core.h"

#include "conf/conf.h"

#include "device/imu/imu.h"
#include "device/mag/mag.h"

#include "mc/filter.h"
#include "mc/mixer.h"
#include "mc/pid.h"

#include "peripheral/bus/bus.h"
#include "peripheral/dma.h"
#include "peripheral/gpio.h"

eSTATUS_t TaskMixerUpdate (uint32_t usCurrentTime) {

    if (STATUS_FAIL (Mixer_Mix (usCurrentTime))) {
        LOG_ERROR ("Failed to mix PID output to actuator outputs");
        return eSTATUS_FAILURE;
    }
    return Mixer_Update (usCurrentTime);
}

eSTATUS_t TaskPIDUpdate (uint32_t usCurrentTime) {

    vPID_t* pPID = PID_GetMutableActivePID ();
    float dt     = (((float)usCurrentTime / 1000.0F) - (float)pPID->msLastUpdateTime) / 1000.0F;
    FlightData_t* pFlightData = &g_FlightData;

    return PID_Update (
    pPID,
    &pFlightData->currentAttitude,
    &pFlightData->targetAttitude,
    &pFlightData->maxAttitude,
    dt,
    &pPID->pidData
    );
}

eSTATUS_t TaskUpdateAttitude (uint32_t usCurrentTime) {

    FlightData_t* pFlightData = &g_FlightData;
    vIMU_t* pIMU              = IMU_GetMutableActiveDevice ();
    vFilter_t* pFilter        = Filter_GetMutableActiveFilter ();
    float dt = ((float)usCurrentTime - (float)pFilter->usLastUpdateTime) / 1000000.0F;
    // TODO: add option for using magnetometer data in the filter
    return Filter_Update (pFilter, &pIMU->accelData, &pIMU->gyroData, NULL, dt, &pFlightData->currentAttitude);
}

eSTATUS_t TaskIMUUpdate (uint32_t usCurrentTime) {

    vIMU_t* pIMUDev = IMU_GetMutableActiveDevice ();
    return IMU_Update (pIMUDev, false, &pIMUDev->accelData, &pIMUDev->gyroData);
}

eSTATUS_t TaskInterCoreSync (uint32_t usCurrentTime) {

    return SyncProcessTasks ();
}