#include <stdbool.h>
#include <stdint.h>

#include "common.h"

#include "cfg/cfg.h"

#include "core/core.h"

#include "aero/flight.h"
#include "aero/pid.h"

#define PID_LIMIT 180.0F

// TODO define better defaults
// clang-format off
CFG_DEFINE (PidCfg_t, PidCfg) = {
    .axis = {
        [AXIS_IDX_YAW] = { .p = 4.0F, .i = 0.0F, .d = 0.0F, .integralLimit = 50.0F },
        [AXIS_IDX_PITCH] = { .p = 4.0F, .i = 0.0F, .d = 0.0F, .integralLimit = 50.0F },
        [AXIS_IDX_ROLL] = { .p = 4.0F, .i = 0.0F, .d = 0.0F, .integralLimit = 50.0F },
    }
};
// clang-format on
FJ_DEFINE_SHARED (PidData_t, e_PidData);

FJ_TESTABLE eSTATUS_t Pid_UpdateAxis (
AXIS_IDX_t axisIdx,
PidAxisCfg_t const* pAxisCfg,
float currentAttitude,
float targetAttitude,
float dt,
PidAxisData_t* pOutAxisData
) {
    float error = targetAttitude - currentAttitude;
    float integral =
    clipf32 (pOutAxisData->prevIntegral + error * dt, -pAxisCfg->integralLimit, pAxisCfg->integralLimit);
    float derivative = (error - pOutAxisData->prevError) / dt;

    pOutAxisData->attitude =
    clipf32 ((pAxisCfg->p * error + pAxisCfg->i * integral - pAxisCfg->d * derivative), -PID_LIMIT, PID_LIMIT) / PID_LIMIT;
    pOutAxisData->prevIntegral = pOutAxisData->integral;
    pOutAxisData->prevError    = pOutAxisData->error;
    pOutAxisData->error        = error;
    pOutAxisData->integral     = integral;
    pOutAxisData->derivative   = derivative;

    return eSTATUS_OK;
}

FJ_TESTABLE eSTATUS_t Pid_UpdateAttitude (
PidCfg_t const* pPidCfg,
PidData_t* pPidData,
float const currentAttitude[AXIS_IDX_COUNT],
float const targetAttitude[AXIS_IDX_COUNT],
uint32_t usCurrentTime,
float attOut[AXIS_IDX_COUNT]
) {

    float dt               = US_TO_SECONDS (usCurrentTime - pPidData->usLastUpdate);
    pPidData->usLastUpdate = usCurrentTime;

    for (uint32_t i = 0; i < AXIS_IDX_COUNT; ++i) {
        PidAxisData_t* pAxisData = &pPidData->axis[i];
        eSTATUS_t status =
        Pid_UpdateAxis ((AXIS_IDX_t)i, &pPidCfg->axis[i], currentAttitude[i], targetAttitude[i], dt, pAxisData);
        RETURN_IF (FJ_FAIL (status), status, "PID axis %d update failed", i);
        attOut[i] = pAxisData->attitude;
    }
    return eSTATUS_OK;
}

eSTATUS_t PidData_Init (void) {

    e_PidData.usLastUpdate = GetMicroseconds ();
    return eSTATUS_OK;
}

eSTATUS_t Pid_Update (uint32_t usCurrentTime) {

    FlightData_t const* pFlightData = FlightData_Get ();
    PidCfg_t const* pPidCfg         = PidCfg_Get ();

    return Pid_UpdateAttitude (
    pPidCfg,
    &e_PidData,
    pFlightData->currentAttitude,
    pFlightData->targetAttitude,
    usCurrentTime,
    e_PidData.pidAttitude
    );
}