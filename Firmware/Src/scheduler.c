#include "scheduler.h"
#include "common.h"
#include "device/device.h"
#include "device/imu/imu.h"
#include "device/mag/mag.h"
#include "fcstate.h"
#include "hal.h"
#include "mc/mc.h"
#include <stdbool.h>


FJ_STATIC void Scheduler_CM4_Loop (void) {

    uint32_t msLastUpdate    = GetMilliseconds ();
    uint32_t const msLogStep = 1000U;
    LOG_INFO ("CM4 Scheduler loop started");

    while (true) {

        SyncProcessTasks ();
        /*
         *
         * TOOD: Should check for controller input here
         *
         */
        if ((GetMilliseconds () - msLastUpdate) >= msLogStep) {
            msLastUpdate = GetMilliseconds ();
            LOG_INFO ("Main loop running");
        }
    }
}

FJ_STATIC void Scheduler_CM7_Loop (void) {

    uint32_t msLastUpdate    = GetMilliseconds ();
    uint32_t const msLogStep = MS_PER_LOG_DATA_UPDATE;
    LOG_INFO ("CM7 Scheduler loop started");

    if (FJ_LOOP_UPDATE_RATE_HZ > 1000U || FJ_LOOP_UPDATE_RATE_HZ < 0U) {
        LOG_ERROR ("Sensor update rate invalid");
    }


    // TODO: TEMPORARY. should be done after receiving a start command
    if (FJ_FAIL (Device_StartAll ())) {
        LOG_ERROR ("Failed to start Device module");
    }

    if (FJ_FAIL (MC_StartAll ())) {
        LOG_ERROR ("Failed to start Motion Control module");
    }
    FC_SET_RUNNING_OP_STATE ();


    eSTATUS_t status      = eSTATUS_SUCCESS;
    FCState_t fcState     = { 0 };
    Vec3f currentAttitude = VEC3F_ZERO ();
    Vec3f targetAttitude  = VEC3F_ZERO ();
    Vec3f maxAttitude     = VEC3F_ZERO ();
    Vec3f pidAttitude     = VEC3F_ZERO ();
    float targetThrottle  = 0.0F;
    float dt              = 0.0F;
    Vec3f accel           = VEC3F_ZERO ();
    Vec3f gyro            = VEC3F_ZERO ();
    Vec3f mag             = VEC3F_ZERO ();
    Vec3f* pMagData       = NULL;

    vIMU_t* pIMUDev    = NULL;
    vMag_t* pMagDev    = NULL;
    vFilter_t* pFilter = NULL;
    vPID_t* pPID       = NULL;

    while (true) {

        /*
         *
         * TODO: Should check for controller input here
         *
         */

        fcState = FCState_GetCopyOfActiveState ();
        if (fcState.opState != eOP_STATE_RUNNING) {
            /*
             * Update msLastUpdate so dt does not get too large.
             */
            msLastUpdate = GetMilliseconds ();
            continue;
        }

        status          = eSTATUS_SUCCESS;
        currentAttitude = fcState.currentAttitude;
        targetAttitude  = fcState.targetAttitude;
        maxAttitude     = fcState.maxAttitude;
        targetThrottle  = fcState.targetThrottle;
        dt              = ((float)GetMilliseconds () - (float)msLastUpdate) / 1000.0F;
        // set to NULL each iteration
        pMagData = NULL;

        pIMUDev = IMU_GetMutableActiveDevice ();
        pMagDev = Mag_GetMutableActiveDevice ();
        pFilter = Filter_GetMutableActiveFilter ();
        pPID    = PID_GetMutableActivePID ();

        status = IMU_Update (pIMUDev, false, &accel, &gyro);
        GOTO_IF (FJ_FAIL (status), error, "Failed to get IMU data");

        // NOTE: Okay if magnetometer data is not available
        if (FJ_OK (Mag_Update (pMagDev, false, &mag))) {
            pMagData = &mag;
        }

        status = Filter_Update (pFilter, &accel, &gyro, pMagData, dt, &currentAttitude);
        GOTO_IF (FJ_FAIL (status), error, "Failed to filter IMU data with Madgwick filter");

        status = PID_Update (pPID, &currentAttitude, &targetAttitude, &maxAttitude, dt, &pidAttitude);
        GOTO_IF (FJ_FAIL (status), error, "Failed to update PID attitude");

        status = Actuators_Update (pidAttitude, targetThrottle);
        GOTO_IF (FJ_FAIL (status), error, "Failed to write actuators");

        status = FC_SET_CURRENT_ATTITUDE (currentAttitude);
        GOTO_IF (FJ_FAIL (status), error, "Failed to set current attitude in FCState");

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

    error:
        msLastUpdate = GetMilliseconds ();
        // Limit loop to sensor update rate
        Delay (1000U / FJ_LOOP_UPDATE_RATE_HZ);
    }
}


void Scheduler_Start (void) {

    if (IS_CM7_ME ()) {
        Scheduler_CM7_Loop ();
    } else {
        Scheduler_CM4_Loop ();
    }
}