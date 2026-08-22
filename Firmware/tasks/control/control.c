#include "tasks/control/control.h"
#include "tasks/control/mixer.h"

#include "common/pid.h"

#include "devices/motors.h"
#include "devices/servos.h"

#include "umsg_guidance.h"
#include "umsg_mission.h"
#include "umsg_nav.h"
#include "umsg_tune.h"

#include "FreeRTOS.h"
#include "task.h"

#include <string.h>

/*
 * This loop paces ITSELF, rather than running once per guidance setpoint.
 *
 * Blocking on guidance made guidance's publish rate the control task's clock -
 * an undocumented coupling that put the actuator loop at the mercy of anything
 * upstream of it in the nav -> guidance chain. All three inputs are latest-value
 * reads now, so a stalled producer costs a stale input rather than a stopped
 * controller.
 *
 * xTaskDelayUntil rather than vTaskDelay: the latter delays for the period
 * AFTER the work, so the real rate is (work + delay) and drifts with load.
 *
 * The period is ticks-per-second divided by the rate, NOT pdMS_TO_TICKS of a
 * millisecond period. At 500 Hz on a 1000 Hz tick both spell 2 ms, but the
 * moment the rate stops being a whole number of milliseconds the pdMS_TO_TICKS
 * form truncates and silently runs fast. The assert keeps the two honest: if
 * either value changes so the rate no longer divides the tick exactly, this
 * fails to build rather than quietly running at something else.
 *
 * 500 Hz rather than 400 because configTICK_RATE_HZ is 1000 and 1000/400 is
 * 2.5 ticks, which cannot be expressed. The pair was 400 Hz on a 2000 Hz tick
 * until the tick was halved to cut the timer ISR load; the PID takes dt per
 * call and the PT1 filters are parameterised by cutoff rather than by a fixed
 * coefficient, so neither needed retuning for the new rate.
 */
#define CONTROL_RATE_HZ      500U
#define CONTROL_PERIOD_TICKS (configTICK_RATE_HZ / CONTROL_RATE_HZ)

_Static_assert ((configTICK_RATE_HZ % CONTROL_RATE_HZ) == 0U,
                "CONTROL_RATE_HZ must divide configTICK_RATE_HZ exactly");

// Plausible bounds on one control interval. dt is still MEASURED rather than
// assumed from the period above - an overrun should scale the integral and
// derivative by the time that actually passed, not by the time intended.
#define CONTROL_DT_MIN_S 0.0005f
#define CONTROL_DT_MAX_S 0.02f

typedef struct {
    umsg_sub_handle_t    guidance_sub;
    umsg_sub_handle_t    nav_sub;
    umsg_sub_handle_t    mission_sub;
    umsg_sub_handle_t    tune_sub;
    uint32_t             usLastUpdateTime;
    umsg_nav_state_t     nav;
    umsg_mission_state_t mission;
    umsg_guidance_setpoints_t sp;
} Control_t;

static Control_t s_Control;

#define PID_CREATE_AXIS(AXIS_NAME) \
    { .p = CFG_PID_##AXIS_NAME##_P, .i = CFG_PID_##AXIS_NAME##_I, .d = CFG_PID_##AXIS_NAME##_D, .integralLimit = CFG_PID_INTEGRAL_LIMIT }

static Pid_t     s_pid = { .axes = {
    [AXIS_IDX_ROLL] = PID_CREATE_AXIS (ROLL),
    [AXIS_IDX_PITCH] = PID_CREATE_AXIS (PITCH),
    [AXIS_IDX_YAW] = PID_CREATE_AXIS (YAW),
    [AXIS_IDX_THROTTLE] = PID_CREATE_AXIS (THROTTLE),
    }
};

static Motors_t s_motors    = { 0 };
static bool     s_prevArmed = false;
static bool     s_prevAltHold = false;

eSTATUS_t Control_Init(void) {
    
    s_Control.guidance_sub      = umsg_guidance_setpoints_subscribe(1, 1);
    s_Control.nav_sub           = umsg_nav_state_subscribe(1, 1);
    s_Control.mission_sub       = umsg_mission_state_subscribe(1, 1);
    s_Control.tune_sub          = umsg_tune_pid_subscribe(1, 1);
    s_Control.usLastUpdateTime  = GetMicroseconds();

    eSTATUS_t status = eSTATUS_SUCCESS;
    do {
        if (STATUS_FAIL (Motors_Init (&s_motors))) {
            LOG_ERROR ("Failed to initialize Motors");
            status = eSTATUS_FAILURE;
            break;
        }

        if (STATUS_FAIL (Servos_Init ())) {
            LOG_ERROR ("Failed to initialize Servos");
            status = eSTATUS_FAILURE;
            break;
        }
        if (STATUS_FAIL (Mixer_Init ())) {
            LOG_ERROR ("Failed to initialize mixer");
            status = eSTATUS_FAILURE;
            break;
        }
        if (STATUS_FAIL (Pid_Init (&s_pid))) {
            LOG_ERROR ("Failed to initialize pid");
            status = eSTATUS_FAILURE;
            break;
        }
    } while(0);

    return status;
}

eSTATUS_t Control_Update(void) {

    /* Latest-value caches, all three: the queues are length 1 (overwrite) and
     * receive() consumes, so keep the last value for iterations with no new
     * message. The setpoint is now cached like the other two - this loop runs
     * on its own clock and must not stall waiting for one. */
    umsg_guidance_setpoints_receive(s_Control.guidance_sub, &s_Control.sp, 0);
    umsg_nav_state_receive(s_Control.nav_sub, &s_Control.nav, 0);
    umsg_mission_state_receive(s_Control.mission_sub, &s_Control.mission, 0);

    bool armed = s_Control.mission.armed != 0;
    if (armed && !s_motors.armed) {
        /*
         * Called every iteration, not on the arm edge: the ESC arms only after a
         * continuous run of zero-throttle frames, and Dshot_Arm emits exactly one
         * per call. This loop is the frame clock, which puts them CONTROL_RATE_HZ
         * apart - 2 ms, inside the 5 ms an ESC tolerates - and keeps the handshake
         * off the CPU. It used to block here for 350 ms.
         *
         * s_motors.armed rather than the s_prevArmed edge, because the condition
         * has to stay true for the whole handshake rather than for one iteration.
         */
        Motors_Arm(&s_motors);
    } else if (!armed && s_prevArmed) {
        Motors_Disarm(&s_motors);
    }
    s_prevArmed = armed;

    umsg_tune_pid_t tune;
    if (umsg_tune_pid_receive(s_Control.tune_sub, &tune, 0) && tune.axis < AXIS_IDX_COUNT) {
        PidAxis_t* pAxis = &s_pid.axes[tune.axis];
        switch (tune.gain) {
            case TUNE_PID_GAIN_KP:             pAxis->p = tune.value; break;
            case TUNE_PID_GAIN_KI:             pAxis->i = tune.value; break;
            case TUNE_PID_GAIN_KD:             pAxis->d = tune.value; break;
            case TUNE_PID_GAIN_INTEGRAL_LIMIT: pAxis->integralLimit = tune.value; break;
            default: break;
        }
    }

    uint32_t usNow = GetMicroseconds();
    float dt = (float)(usNow - s_Control.usLastUpdateTime) / 1000000.0f;
    s_Control.usLastUpdateTime = usNow;

    // The loop is paced by queued IMU samples, so a drained burst can put two
    // iterations in the same microsecond (dt == 0) and a stall can produce an
    // arbitrarily long one. Neither is a real control interval; bound dt rather
    // than let it scale the integral and derivative terms.
    dt = clipf32(dt, CONTROL_DT_MIN_S, CONTROL_DT_MAX_S);

    /*
     * Disarmed, the loop does no work and every axis is held reset.
     *
     * It used to run in full and gate only the two actuator WRITES at the
     * bottom, which meant the rate PIDs spent the whole pre-arm hold
     * integrating a stick deflection the vehicle cannot answer - nothing moves,
     * so the error never reduces and the integral is pure stored lag. It landed
     * as a full-differential twitch in the first frames of flight: two opposite
     * 1.000/0.050 motor commands about 15 ms after the props spun.
     *
     * Holding the state reset here rather than clearing it on the arm edge is
     * the difference between the loop never winding up and unwinding it once
     * afterwards. It also keeps the mixer from leaving live actuator commands
     * in g_Mixer that nothing is executing.
     *
     * Placement is deliberate. The message reads, the arm edge and the TUNE
     * handler are all upstream: a bench `set_pid` over the shell has to work
     * while disarmed, which is precisely when it would be used. dt is taken
     * above too, so the first armed interval is one period rather than however
     * long the vehicle sat on the ground.
     */
    /* Arming counts as disarmed here: the ESC is still being handshaken, so the
     * loop holds its state reset and writes no actuators until it is live. */
    if (!armed || !s_motors.armed) {
        for (uint32_t i = 0; i < AXIS_IDX_COUNT; ++i) {
            Pid_ResetAxis (&s_pid.axes[i]);
        }
        /* So the altitude-hold entry below sees a genuine entry on arming,
         * rather than carrying s_prevAltHold across a disarm from the last
         * flight and skipping its own reset. */
        s_prevAltHold = false;
        return eSTATUS_SUCCESS;
    }

    // Rate PID: guidance w[] is in rad/s, nav gyro[] is in deg/s from the IMU.
    // Convert targets to deg/s to match the sensor units.
    float pid_data[AXIS_IDX_COUNT];
    float targets[3] = {
        RAD2DEG(s_Control.sp.w[0]),
        RAD2DEG(s_Control.sp.w[1]),
        RAD2DEG(s_Control.sp.w[2]),
    };

    // Pid_UpdateAxis already returns the normalised mixer command, clipped to
    // CFG_PID_MIN/MAX_VALUE (-1..+1). Re-clipping to a rate limit and dividing
    // by it - as this did - made the +/-5 PID clip bind first and capped every
    // axis at 2.8% of actuator travel.
    Pid_t* pPid = &s_pid;
    for (uint32_t i = 0; i < 3U; ++i) {
        pid_data[i] = Pid_UpdateAxis(&pPid->axes[i], s_Control.nav.gyro[i], targets[i], dt);
    }
    /*
     * Throttle. In MANUAL, vel_b[2] is the normalised collective the mixer
     * consumes and passes straight through. In ALTITUDE_HOLD it is a TARGET
     * ALTITUDE in metres, and the throttle PID trims around a hover
     * feedforward to reach it.
     *
     * This axis is deliberately not folded into the rate loop above: those
     * three close on gyro, and there is no gyro for the vertical axis. It
     * closes on the nav altitude estimate instead.
     *
     * NAV_VALID_BARO_ALT is checked here rather than trusted from guidance:
     * the estimate can drop out between the two tasks, and closing a loop on
     * an altitude that no longer exists commands full throttle at whatever the
     * stale error was.
     */
    bool altHold = (s_Control.nav.valid & NAV_VALID_BARO_ALT) != 0U;

    /* Entering the mode - or arming into it - starts from a clean integrator.
     * A stale one carries in the wind-up from the last engagement and spends
     * the first second of the hold unwinding it. */
    if (altHold && (!s_prevAltHold || !s_prevArmed)) {
        Pid_ResetAxis (&s_pid.axes[AXIS_IDX_THROTTLE]);
    }
    s_prevAltHold = altHold;

    if (altHold) {
        float alt = -s_Control.nav.pos_ned[2];   // NED, down positive
        float vz  = -s_Control.nav.vel_ned[2];   // climb rate, up positive
        float trim = Pid_UpdateAxis(&pPid->axes[AXIS_IDX_THROTTLE], alt, s_Control.sp.vel_b[2], dt);
        /*
         * Vertical damping, on the climb-rate ERROR rather than on vz itself.
         * It lives outside the PID because it is derived from a different (and
         * far cleaner) estimate than the one the PID closes on - see
         * CFG_ALT_HOLD_VZ_DAMPING.
         *
         * Subtracting the COMMANDED rate is what keeps this a damper rather
         * than a brake. Against raw vz it opposed the climb the pilot had just
         * asked for: at the full 1 m/s stick the term contributes -0.5 of
         * throttle, which P and I then have to manufacture back, so the vehicle
         * settled ~0.4 m under the target with the integrator pinned at
         * CFG_PID_INTEGRAL_LIMIT for the whole climb - and ballooned when the
         * stick centred, because the pinned integrator was left with nothing to
         * cancel. Zeroed in a tracked climb, the term now only fights the part
         * of vz the pilot did not ask for, and carries the climb-rate
         * feedforward for free.
         *
         * sp.climb_rate is d(sp.vel_b[2])/dt by construction (guidance.c); that
         * is the invariant this depends on, not merely a convenience.
         */
        trim -= CFG_ALT_HOLD_VZ_DAMPING * (vz - s_Control.sp.climb_rate);

        /*
         * Tilt compensation. Lift is the VERTICAL component of thrust, so a
         * banked or pitched airframe loses it as cos(tilt) - 13% at the 30 deg
         * angle limit (see CFG_ANGLE_MAX_DEG). Without this the integrator has
         * to discover that deficit and cancel it, and it has only
         * CFG_PID_INTEGRAL_LIMIT (0.3) of authority to do it with.
         *
         * cosTilt is the bottom-right element of the body->NED rotation, the
         * same one Nav_VerticalAccelUp forms its row from. Multiplying the WHOLE
         * command - feedforward included, not just the trim - is the point: it
         * is the hover thrust that is being projected away.
         *
         * Floored at 0.5 (60 deg, 2x boost) so an extreme attitude cannot ask
         * for unbounded throttle. Written as !(x > 0.5) rather than (x < 0.5) so
         * a NaN quaternion floors instead of propagating - same idiom as the dt
         * guard in pid.c.
         *
         * NOT the same thing as the mixer's ABS(pitch) collective term, which
         * gives back lift lost to ROTOR tilt relative to the airframe. Bank of
         * the airframe itself is what this covers.
         */
        float q2      = s_Control.nav.quat[1];
        float q3      = s_Control.nav.quat[2];
        float cosTilt = 1.0f - (2.0f * ((q2 * q2) + (q3 * q3)));
        if (!(cosTilt > 0.5f)) {
            cosTilt = 0.5f;
        }

        pid_data[AXIS_IDX_THROTTLE] =
            clipf32((CFG_HOVER_THROTTLE + trim) / cosTilt, 0.0f, 1.0f);
    } else {
        pid_data[AXIS_IDX_THROTTLE] = s_Control.sp.vel_b[2];
    }

    Mixer_MixMotors(&g_Mixer, pid_data, g_Mixer.motorOutputs);
    memset(g_Mixer.servoOutputs, 0, sizeof(g_Mixer.servoOutputs));
    Mixer_MixServos(&g_Mixer, pid_data, g_Mixer.servoOutputs);

    eSTATUS_t status = Servos_Write(g_Mixer.servoOutputs);
    if (STATUS_FAIL(status)) {
        LOG_ERROR("Control: failed to write servo outputs");
        return status;
    }

    status = Motors_Write(&s_motors, g_Mixer.motorOutputs);
    if (STATUS_FAIL(status)) {
        LOG_ERROR("Control: failed to write motor outputs");
        return status;
    }

    return eSTATUS_SUCCESS;
}

void Control_Task(void* args) {
    (void)args;
    Control_Init();
    TickType_t lastWake = xTaskGetTickCount();
    while (1) {
        Control_Update();
        xTaskDelayUntil (&lastWake, CONTROL_PERIOD_TICKS);
    }
}
