#ifndef MC_PID_H
#define MC_PID_H

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

void Pid_LogData (Pid_t* pPid);

eSTATUS_t Pid_Init (Pid_t* pOutPid);

float Pid_UpdateAxis (PidAxis_t* pAxis, float current, float target, float dt);


#endif // MC_PID_H