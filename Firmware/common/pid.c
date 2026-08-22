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

    /* Set here rather than in PID_CREATE_AXIS because that macro is duplicated
     * in control.c for a static initialiser; a field added to only one copy is
     * a trap. This runs after those assignments and covers every axis. */
    for (uint32_t i = 0; i < AXIS_IDX_COUNT; ++i) {
        pOutPid->axes[i].dLpf.cfg.cutoffHz = CFG_PID_DTERM_LPF_HZ;
        LowPassFilter_Init (&pOutPid->axes[i].dLpf);
    }
    return eSTATUS_SUCCESS;
}

void Pid_ResetAxis (PidAxis_t* pAxis) {

    if (!pAxis) {
        return;
    }
    pAxis->prevIntegral       = 0.0F;
    pAxis->hasPrevMeasurement = false;
    /* The D filter carries state too, and a stale one is the same false-spike
     * problem hasPrevMeasurement guards against - it just arrives spread over a
     * time constant instead of in one frame. Init clears state and leaves the
     * configured cutoff alone. */
    LowPassFilter_Init (&pAxis->dLpf);
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
    /*
     * Output-saturation anti-windup, by conditional integration.
     *
     * The I-term clamp below bounds the integrator's own CONTRIBUTION, but on
     * its own it cannot stop the integral growing while the total output is
     * already pinned at CFG_PID_MIN/MAX_VALUE. The actuator cannot deliver more
     * than full travel, so every error-second accumulated past that point is
     * pure stored lag that has to be unwound before the loop can respond in the
     * other direction at all.
     *
     * That is not theoretical here. Once the FDM was given realistic servo rate
     * limits, a saturated rate loop with no anti-windup kept integrating through
     * the saturation, and the recovery it eventually commanded was large enough
     * to saturate the other way - a limit cycle it never left.
     *
     * The rule: integrate unless the output is already saturated AND this
     * error would push it further out. Errors that would bring it back toward
     * the linear region are always integrated, so the term recovers immediately
     * rather than waiting out the accumulated excess.
     */
    float provisional = (pAxis->p * error) + (pAxis->i * pAxis->prevIntegral);
    bool blockedHigh  = (provisional >= CFG_PID_MAX_VALUE) && (error > 0.0F);
    bool blockedLow   = (provisional <= CFG_PID_MIN_VALUE) && (error < 0.0F);

    float integral = (blockedHigh || blockedLow) ? pAxis->prevIntegral
                                                 : pAxis->prevIntegral + (error * dt);
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
    /*
     * Low-pass the MEASUREMENT and difference that, rather than differencing the
     * raw signal and low-passing the result. For a linear filter the two are
     * algebraically the same, but this ordering never forms the raw difference
     * at all, and that difference - divided by a dt as small as 0.5 ms - is the
     * large intermediate. It is also the order Betaflight uses.
     *
     * prevMeasurement therefore holds the FILTERED value. Both sides of the
     * subtraction have to come from the same signal; mixing a filtered sample
     * with a raw one differences two different signals and reports the filter's
     * own lag as vehicle motion.
     *
     * With cutoffHz <= 0 this is exact pass-through, so the axis behaves
     * bit-for-bit as it did before the filter existed.
     */
    float dMeasurement = LowPassFilter_Update (&pAxis->dLpf, current, dt);

    float derivative = 0.0F;
    if (pAxis->hasPrevMeasurement) {
        derivative = (dMeasurement - pAxis->prevMeasurement) / dt;
    }
    pAxis->hasPrevMeasurement = true;

    pAxis->prevIntegral    = integral;
    pAxis->prevMeasurement = dMeasurement;

    return clipf32 ((pAxis->p * error) + iTerm - (pAxis->d * derivative), CFG_PID_MIN_VALUE, CFG_PID_MAX_VALUE);
}

void Pid_LogData (Pid_t* pPid) {
    
    LOG_4_FLOATS(LOG_DATA_TYPE_PID_ATTITUDE, 
        roll, pPid->data[AXIS_IDX_ROLL], 
        pitch, pPid->data[AXIS_IDX_PITCH], 
        yaw, pPid->data[AXIS_IDX_YAW], 
        throttle, pPid->data[AXIS_IDX_THROTTLE]);
}