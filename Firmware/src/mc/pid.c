#include "target.h"

#include "core/core.h"

#include "mc/pid.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define PID_CREATE_AXIS(AXIS_NAME) \
    { .p = CFG_PID_##AXIS_NAME##_P, .i = CFG_PID_##AXIS_NAME##_I, .d = CFG_PID_##AXIS_NAME##_D, .integralLimit = CFG_PID_INTEGRAL_LIMIT }
// clang-format off
FJ_DEFINE_SHARED (Pid_t, g_Pid) = { .axes = {
    [AXIS_IDX_ROLL] = PID_CREATE_AXIS (ROLL),
    [AXIS_IDX_PITCH] = PID_CREATE_AXIS (PITCH),
    [AXIS_IDX_YAW] = PID_CREATE_AXIS (YAW),
    [AXIS_IDX_THROTTLE] = PID_CREATE_AXIS (THROTTLE),
    } 
};
//clang-format on

eSTATUS_t Pid_Init_(Pid_t* pOutPid) {

    if(!pOutPid) {
        return eSTATUS_FAILURE;
    }
    pOutPid->usLastUpdateTime = GetMicroseconds();
    return eSTATUS_SUCCESS;
}

float Pid_UpdateAxis_ (PidAxis_t* pAxis, float current, float target, float dt) {

    float error = target - current;
    float integral = clipf32 (pAxis->prevIntegral + (error * dt), -pAxis->integralLimit, pAxis->integralLimit);
    float derivative = (error - pAxis->prevError) / dt;

    pAxis->prevIntegral = integral;
    pAxis->prevError    = error;

    return clipf32 ((pAxis->p * error) + (pAxis->i * integral) - (pAxis->d * derivative), CFG_PID_MIN_VALUE, CFG_PID_MAX_VALUE);
}

void Pid_LogData_ (Pid_t* pPid) {
    
    LOG_4_FLOATS(LOG_DATA_TYPE_PID_ATTITUDE, 
        roll, pPid->data[AXIS_IDX_ROLL], 
        pitch, pPid->data[AXIS_IDX_PITCH], 
        yaw, pPid->data[AXIS_IDX_YAW], 
        throttle, pPid->data[AXIS_IDX_THROTTLE]);
}