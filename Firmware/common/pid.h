#ifndef MC_PID_H
#define MC_PID_H

#include "target.h"

#include "core/core.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

typedef struct {
    float p, i, d;
    /* Maximum magnitude the I-term may contribute to the OUTPUT, in the same
     * normalised units the controller returns (so 0.3 lets the integrator claim
     * at most 30% of actuator travel). It used to bound the raw accumulated
     * error instead, which made the ceiling depend on the I gain. */
    float integralLimit;
    float prevMeasurement;
    float prevIntegral;
    bool  hasPrevMeasurement;
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