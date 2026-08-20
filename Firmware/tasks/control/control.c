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

#include <string.h>

// Plausible bounds on one control interval: the loop is IMU-paced, so anything
// faster than 2 kHz is a duplicated iteration and anything slower than 50 Hz is
// a stall.
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

    umsg_guidance_setpoints_t sp;
    if (!umsg_guidance_setpoints_receive(s_Control.guidance_sub, &sp, portMAX_DELAY)) {
        return eSTATUS_FAILURE;
    }

    // Latest-value caches: these queues are length 1 (overwrite), and receive()
    // consumes, so keep the last value for iterations with no new message.
    umsg_nav_state_receive(s_Control.nav_sub, &s_Control.nav, 0);
    umsg_mission_state_receive(s_Control.mission_sub, &s_Control.mission, 0);

    bool armed = s_Control.mission.armed != 0;
    if (armed && !s_prevArmed) {
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

    // Rate PID: guidance w[] is in rad/s, nav gyro[] is in deg/s from the IMU.
    // Convert targets to deg/s to match the sensor units.
    float pid_data[AXIS_IDX_COUNT];
    float targets[3] = {
        RAD2DEG(sp.w[0]),
        RAD2DEG(sp.w[1]),
        RAD2DEG(sp.w[2]),
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
     * NAV_VALID_BARO_ALT is re-checked here rather than trusted from guidance:
     * the estimate can drop out between the two tasks, and closing a loop on
     * an altitude that no longer exists commands full throttle at whatever the
     * stale error was.
     */
    bool altHold = (s_Control.mission.mode == EMISSION_MODE_ALTITUDE_HOLD) &&
                   ((s_Control.nav.valid & NAV_VALID_BARO_ALT) != 0U);

    /* Entering the mode - or arming into it - starts from a clean integrator.
     * A stale one carries in the wind-up from the last engagement and spends
     * the first second of the hold unwinding it. */
    if (altHold && (!s_prevAltHold || !s_prevArmed)) {
        s_pid.axes[AXIS_IDX_THROTTLE].prevIntegral       = 0.0f;
        s_pid.axes[AXIS_IDX_THROTTLE].hasPrevMeasurement = false;
    }
    s_prevAltHold = altHold;

    if (altHold) {
        float alt = -s_Control.nav.pos_ned[2];   // NED, down positive
        float vz  = -s_Control.nav.vel_ned[2];   // climb rate, up positive
        float trim = Pid_UpdateAxis(&pPid->axes[AXIS_IDX_THROTTLE], alt, sp.vel_b[2], dt);
        /* Damping term, applied outside the PID because it is derived from a
         * different (and far cleaner) estimate than the one the PID closes on
         * - see CFG_ALT_HOLD_VZ_DAMPING. */
        trim -= CFG_ALT_HOLD_VZ_DAMPING * vz;
        pid_data[AXIS_IDX_THROTTLE] = clipf32(CFG_HOVER_THROTTLE + trim, 0.0f, 1.0f);
    } else {
        pid_data[AXIS_IDX_THROTTLE] = sp.vel_b[2];
    }

    Mixer_MixMotors(&g_Mixer, pid_data, g_Mixer.motorOutputs);
    memset(g_Mixer.servoOutputs, 0, sizeof(g_Mixer.servoOutputs));
    Mixer_MixServos(&g_Mixer, pid_data, g_Mixer.servoOutputs);

    if (!armed) {
        return eSTATUS_SUCCESS;
    }

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
    while (1) {
        Control_Update();
    }
}
