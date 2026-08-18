#include "target.h"

#include "core/core.h"

#include "common/pid.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define PID_CREATE_AXIS(AXIS_NAME) \
    { .p = CFG_PID_##AXIS_NAME##_P, .i = CFG_PID_##AXIS_NAME##_I, .d = CFG_PID_##AXIS_NAME##_D, .integralLimit = CFG_PID_INTEGRAL_LIMIT }

/*
 * Smallest I gain the back-calculation below will divide by.
 *
 * A magnitude test rather than `i != 0.0F`, for two reasons:
 *
 *   - An i small enough that integralLimit / i overflows binary32 stores +inf
 *     into the integral. The next cycle recomputes iTerm = i * inf = inf, clamps
 *     it, and stores inf again - the axis latches a saturated I-term for the
 *     rest of the flight. The window is narrow (|i| < ~1e-39 at the default
 *     limit) but reachable: Control_Update assigns tune.value straight to the
 *     gain with no validation, so a shell set_pid can put any float there.
 *   - `i != 0.0F` is TRUE when i is NaN, which would feed NaN into the stored
 *     integral and from there into the output. The comparison below is FALSE for
 *     NaN, so a NaN gain resets the integrator instead of poisoning it.
 *
 * 1e-9 is five orders of magnitude below the smallest configured gain
 * (CFG_PID_YAW_I = 2.8e-4), so no real tune can reach it, and it keeps
 * integralLimit / i finite for any limit up to ~3e29.
 */
#define PID_MIN_I_GAIN 1.0e-9F

static bool Pid_IsIGainUsable (float i) {
    /* Two-sided rather than fabsf() so pid.c does not take a math.h dependency
     * for one comparison. Also false for NaN, which is the point. */
    return (i > PID_MIN_I_GAIN) || (i < -PID_MIN_I_GAIN);
}

eSTATUS_t Pid_Init(Pid_t* pOutPid) {

    if(!pOutPid) {
        return eSTATUS_FAILURE;
    }
    pOutPid->axes[AXIS_IDX_ROLL]     = (PidAxis_t)PID_CREATE_AXIS (ROLL);
    pOutPid->axes[AXIS_IDX_PITCH]    = (PidAxis_t)PID_CREATE_AXIS (PITCH);
    pOutPid->axes[AXIS_IDX_YAW]      = (PidAxis_t)PID_CREATE_AXIS (YAW);
    pOutPid->axes[AXIS_IDX_THROTTLE] = (PidAxis_t)PID_CREATE_AXIS (THROTTLE);
    pOutPid->usLastUpdateTime        = GetMicroseconds();
    return eSTATUS_SUCCESS;
}

float Pid_UpdateAxis (PidAxis_t* pAxis, float current, float target, float dt) {

    float error = target - current;

    /*
     * Reject a non-positive or NaN dt before dividing by it.
     *
     * GetMicroseconds() has 1 us resolution and this loop is paced by queued
     * IMU samples, so when a burst drains, two iterations land in the same
     * microsecond and dt is exactly 0. The derivative then evaluates to +/-inf
     * (or NaN when the rate has not changed either), and the mixer forwards it
     * to the actuators: inf clips to full deflection, NaN propagates all the way
     * into the flight model. That is what put the SIL into NaN about a second
     * after arming.
     *
     * Nothing has happened over a zero interval, so hold the proportional
     * response plus the existing integral and leave the state untouched. The
     * stored integral is already clamped so i * prevIntegral is in range.
     */
    if (!(dt > 0.0F)) {
        return clipf32 ((pAxis->p * error) + (pAxis->i * pAxis->prevIntegral),
                        CFG_PID_MIN_VALUE, CFG_PID_MAX_VALUE);
    }

    /*
     * Integrate, then clamp the I-term's CONTRIBUTION and push the clamp back
     * into the stored integral so it cannot wind up while held at the limit.
     *
     * The clamp used to bound the raw accumulated error, which made the actual
     * authority depend on the I gain: at the old limit of 25 error-seconds with
     * i = 0.00167 the integrator could only ever reach 4% of travel.
     */
    float integral = pAxis->prevIntegral + (error * dt);
    float iTerm    = pAxis->i * integral;
    if (iTerm > pAxis->integralLimit) {
        iTerm    = pAxis->integralLimit;
        integral = Pid_IsIGainUsable (pAxis->i) ? (iTerm / pAxis->i) : 0.0F;
    } else if (iTerm < -pAxis->integralLimit) {
        iTerm    = -pAxis->integralLimit;
        integral = Pid_IsIGainUsable (pAxis->i) ? (iTerm / pAxis->i) : 0.0F;
    }

    /*
     * Derivative on the MEASUREMENT, not on the error.
     *
     * Differentiating the error made this term destabilising. With a steady
     * setpoint d(error)/dt = -d(rate)/dt, so the "- d * derivative" below
     * evaluated to +d * d(rate)/dt: positive feedback on exactly the motion the
     * D term exists to damp. Differentiating the measurement restores the
     * damping sign and, as a bonus, removes the derivative kick that a stick
     * step used to inject.
     *
     * Skip the first sample - prevMeasurement is meaningless then, and current/dt
     * would be a large false spike.
     */
    float derivative = 0.0F;
    if (pAxis->hasPrevMeasurement) {
        derivative = (current - pAxis->prevMeasurement) / dt;
    }
    pAxis->hasPrevMeasurement = true;

    pAxis->prevIntegral    = integral;
    pAxis->prevMeasurement = current;

    return clipf32 ((pAxis->p * error) + iTerm - (pAxis->d * derivative), CFG_PID_MIN_VALUE, CFG_PID_MAX_VALUE);
}

void Pid_LogData (Pid_t* pPid) {
    
    LOG_4_FLOATS(LOG_DATA_TYPE_PID_ATTITUDE, 
        roll, pPid->data[AXIS_IDX_ROLL], 
        pitch, pPid->data[AXIS_IDX_PITCH], 
        yaw, pPid->data[AXIS_IDX_YAW], 
        throttle, pPid->data[AXIS_IDX_THROTTLE]);
}