#include <stdio.h>

#include "control.h"
#include "fcstate.h"
#include "flight.h"
#include "hal.h"
#include "task.h"

#include "core/core.h"

#include "conf/conf.h"

#include "device/device.h"
#include "device/imu/imu.h"
#include "device/mag/mag.h"

#include "mc/actuators.h"
#include "mc/filter.h"
#include "mc/mc.h"
#include "mc/pid.h"

#include "peripheral/bus/bus.h"
#include "peripheral/dma.h"
#include "peripheral/gpio.h"

eSTATUS_t TaskMixerUpdate (uint32_t usCurrentTime) {

    FlightData_t* pFlightData = &g_FlightData;
    vPID_t* pPID              = PID_GetMutableActivePID ();
    return Actuators_Update (pPID->pidData, pFlightData->targetThrottle);
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

void TaskMotionControlUpdate (void) {

    uint32_t msLastUpdate    = GetMilliseconds ();
    uint32_t const msLogStep = MS_PER_LOG_DATA_UPDATE;
    LOG_INFO ("Motion control update task started");

    if (FJ_LOOP_UPDATE_RATE_HZ > 1000U || FJ_LOOP_UPDATE_RATE_HZ < 0U) {
        LOG_ERROR ("Sensor update rate invalid");
    }

    // TODO: TEMPORARY. should be done after receiving a start command
    if (Device_StartAll () != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to start Device module");
    }

    if (MC_StartAll () != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to start Motion Control module");
    }
    FC_SET_RUNNING_OP_STATE ();

    eSTATUS_t status      = eSTATUS_SUCCESS;
    FCState_t fcState     = { 0 };
    Vec3f currentAttitude = { 0.0F };
    Vec3f targetAttitude  = { 0.0F };
    Vec3f maxAttitude     = { 0.0F };
    Vec3f pidAttitude     = { 0.0F };
    float targetThrottle  = 0.0F;
    float dt              = 0.0F;
    Vec3f accel           = { 0.0F };
    Vec3f gyro            = { 0.0F };
    Vec3f mag             = { 0.0F };
    Vec3f* pMagData       = NULL;

    while (true) {

        fcState = FCState_GetCopyOfActiveState ();
        if (fcState.opState != eOP_STATE_RUNNING) {
            /*
             * Update msLastUpdate so dt does not get too large.
             */
            msLastUpdate = GetMilliseconds ();
            // Limit state checks to 1000Hz
            Delay (1);
            continue;
        }
        /*
         * TODO: notifications from IMU started breaking FreeRTOS.
         * I am hitting this assert sometimes: configASSERT( listLIST_ITEM_CONTAINER( &( pxTCB->xEventListItem ) ) == NULL in tasks.c
         */
        // ulTaskNotifyTake (pdTRUE, pdMS_TO_TICKS (1000));
        vIMU_t* pIMUDev    = IMU_GetMutableActiveDevice ();
        vMag_t* pMagDev    = Mag_GetMutableActiveDevice ();
        vFilter_t* pFilter = Filter_GetMutableActiveFilter ();
        vPID_t* pPID       = PID_GetMutableActivePID ();
        status             = eSTATUS_SUCCESS;
        currentAttitude    = fcState.currentAttitude;
        targetAttitude     = fcState.targetAttitude;
        maxAttitude        = fcState.maxAttitude;
        targetThrottle     = fcState.targetThrottle;
        dt                 = ((float)GetMilliseconds () - (float)msLastUpdate) / 1000.0F;
        // set to NULL each iteration
        pMagData = NULL;

        if (STATUS_FAIL (IMU_Update (pIMUDev, false, &accel, &gyro))) {
            LOG_ERROR ("Failed to get IMU data");
            continue;
        }

        // NOTE: Okay if magnetometer data is not available
        if (STATUS_OK (Mag_Update (pMagDev, false, &mag))) {
            pMagData = &mag;
        }

        status = Filter_Update (pFilter, &accel, &gyro, pMagData, dt, &currentAttitude);
        if (STATUS_FAIL (status)) {
            LOG_ERROR ("Failed to filter IMU data with Madgwick filter");
            continue;
        }


        status = PID_Update (pPID, &currentAttitude, &targetAttitude, &maxAttitude, dt, &pidAttitude);
        if (STATUS_FAIL (status)) {
            LOG_ERROR ("Failed to update PID attitude");
            continue;
        }

        status = Actuators_Update (pidAttitude, targetThrottle);
        if (STATUS_FAIL (status)) {
            LOG_ERROR ("Failed to write actuators");
            continue;
        }

        if (FC_SET_CURRENT_ATTITUDE (currentAttitude) == false) {
            LOG_ERROR ("Failed to set current attitude in FCState");
            continue;
        }

        if ((GetMilliseconds () - msLastUpdate) >= msLogStep) {

            Vec3f a   = accel;
            Vec3f g   = gyro;
            Vec3f ca  = currentAttitude;
            Vec3f pid = pidAttitude;
            pid.roll *= maxAttitude.roll;
            pid.pitch *= maxAttitude.pitch;
            pid.yaw *= maxAttitude.yaw;

            // portENTER_CRITICAL ();
            LOG_DATA_IMU_DATA (a, g);
            LOG_DATA_CURRENT_ATTITUDE (ca);
            LOG_DATA_CURRENT_PID_ATTITUDE (pid);
            ActuatorsLogData ();
            // portEXIT_CRITICAL ();
        }

        msLastUpdate = GetMilliseconds ();
        // Limit loop to sensor update rate
        Delay (1000U / FJ_LOOP_UPDATE_RATE_HZ);
    }
}