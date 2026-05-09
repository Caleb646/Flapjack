#ifndef MC_PID_H
#define MC_PID_H

#include "flight.h"
#include "target.h"

#include "core/core.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

typedef struct {
    float p, i, d;
    float integralLimit;
    float prevError;
    float prevIntegral;
} PidAxis_t;

typedef struct {

    PidAxis_t axes[AXIS_IDX_COUNT];
    float data[AXIS_IDX_COUNT];
    uint32_t usLastUpdateTime;

} Pid_t;

FJ_DECLARE_SHARED (Pid_t, g_Pid);

static inline Pid_t* Pid_Get (void) {
    return &g_Pid;
};

void Pid_LogData_ (Pid_t* pPid);
static inline void Pid_LogData (void) {
    Pid_LogData_ (&g_Pid);
}

eSTATUS_t Pid_Init_ (Pid_t* pOutPid);
static inline eSTATUS_t Pid_Init (void) {
    return Pid_Init_ (&g_Pid);
}

float Pid_UpdateAxis_ (PidAxis_t* pAxis, float current, float target, float dt);
eSTATUS_t Pid_Update_ (Pid_t* pPid, Flight_t* pFlightData, uint32_t usCurrentTime, uint32_t usDeltaTime);
static inline eSTATUS_t Pid_Update (uint32_t usCurrentTime, uint32_t usDeltaTime) {
    return Pid_Update_ (Pid_Get (), Fc_Get (), usCurrentTime, usDeltaTime);
}


#endif // MC_PID_H