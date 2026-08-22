#ifndef MC_PID_H
#define MC_PID_H

#include "target.h"

#include "common/filter.h"
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
    /* The FILTERED measurement from last iteration, not the raw one - both
     * sides of the derivative's difference have to come from the same signal. */
    float prevMeasurement;
    float prevIntegral;
    bool  hasPrevMeasurement;
    /* D path only. P and I deliberately close on the raw measurement: this
     * filter buys noise rejection with phase lag, and the D term is the only
     * one whose noise gain rises with frequency, so it is the only one worth
     * paying for. Filtering the shared input instead would spend the same phase
     * on P, which is the dominant term. */
    LowPassFilter_t dLpf;
} PidAxis_t;

typedef struct {

    PidAxis_t axes[AXIS_IDX_COUNT];
    float data[AXIS_IDX_COUNT];
    uint32_t usLastUpdateTime;

} Pid_t;

void Pid_LogData (Pid_t* pPid);

eSTATUS_t Pid_Init (Pid_t* pOutPid);

/*
 * Drop every piece of history one axis carries: the integrator, the derivative's
 * previous sample, and the D filter's state. Callers reset on arming and on
 * mode entry; going through here rather than poking the fields keeps a newly
 * added piece of state from being silently left stale at those sites.
 */
void Pid_ResetAxis (PidAxis_t* pAxis);

float Pid_UpdateAxis (PidAxis_t* pAxis, float current, float target, float dt);


#endif // MC_PID_H