#include "tasks/guidance/guidance.h"
#include "target.h"
#include "core/core.h"
#include "drivers/rx/rx.h"
#include "umsg_nav.h"
#include "umsg_mission.h"
#include "umsg_rc.h"
#include "umsg_guidance.h"

#include "FreeRTOS.h"

#include <math.h>
#include <stdbool.h>

#define GUIDANCE_MAX_RATE_RAD_S 3.14159265f

/* Same bounds control uses on its own dt, and for the same reason: this
 * task is paced by nav, so a drained burst gives 0 and a stall gives an
 * arbitrarily long interval. Neither should scale the target walk. */
#define GUIDANCE_DT_MIN_S 0.0005f
#define GUIDANCE_DT_MAX_S 0.02f

typedef struct {
    umsg_sub_handle_t    nav_sub;
    umsg_sub_handle_t    mission_sub;
    umsg_sub_handle_t    rc_sub;
    umsg_mission_state_t mission;
    umsg_rc_input_t      rc;
    // Altitude-hold target, in metres above the nav datum. Held across
    // iterations; re-captured on every entry into the mode.
    float                altTarget;
    bool                 altHoldActive;
    uint32_t             usLastUpdateTime;
} Guidance_t;

static Guidance_t s_Guidance;


eSTATUS_t Guidance_Init(void) {
    s_Guidance.nav_sub     = umsg_nav_state_subscribe(1, 4);
    s_Guidance.mission_sub = umsg_mission_state_subscribe(1, 1);
    s_Guidance.rc_sub      = umsg_rc_input_subscribe(1, 1);
    s_Guidance.altTarget       = 0.0f;
    s_Guidance.altHoldActive   = false;
    s_Guidance.usLastUpdateTime = GetMicroseconds();
    return eSTATUS_SUCCESS;
}

eSTATUS_t Guidance_Update(void) {
    umsg_nav_state_t nav;
    if (!umsg_nav_state_receive(s_Guidance.nav_sub, &nav, portMAX_DELAY)) {
        return eSTATUS_FAILURE;
    }

    // Latest-value caches: mission and RC publish slower than nav drives this
    // task, and receive() consumes, so keep the last value between messages.
    umsg_mission_state_receive(s_Guidance.mission_sub, &s_Guidance.mission, 0);
    umsg_rc_input_receive(s_Guidance.rc_sub, &s_Guidance.rc, 0);

    uint32_t usNow = GetMicroseconds();
    float dt = (float)(usNow - s_Guidance.usLastUpdateTime) / 1000000.0f;
    s_Guidance.usLastUpdateTime = usNow;
    dt = clipf32(dt, GUIDANCE_DT_MIN_S, GUIDANCE_DT_MAX_S);

    /* Zero-initialised: publishing an uninitialised local would put stack
     * garbage on the rate and throttle setpoints, and the mixer forwards
     * whatever it is given straight to the actuators. Every field below is
     * assigned unconditionally now, so this is belt-and-braces rather than a
     * live fallback - but it is the cheap half of the pair. */
    umsg_guidance_setpoints_t sp = { 0 };
    sp.quat[0] = 1.0f;
    sp.quat[1] = 0.0f;
    sp.quat[2] = 0.0f;
    sp.quat[3] = 0.0f;

    /*
     * Purely a sensor gate. Without a usable altitude there is nothing to
     * capture a target from, so the throttle stick falls back to PASSTHROUGH -
     * raw stick straight to the mixer. A centred stick there is 1500 us -> 0.5,
     * which is hover thrust, so the fallback holds rather than drops.
     */
    bool altHold = (nav.valid & NAV_VALID_BARO_ALT) != 0U;

    float roll  = ((float)s_Guidance.rc.channels[RC_CHANNEL_IDX_ROLL]     - 1500.0f) / 500.0f;
    float pitch = ((float)s_Guidance.rc.channels[RC_CHANNEL_IDX_PITCH]    - 1500.0f) / 500.0f;
    float yaw   = ((float)s_Guidance.rc.channels[RC_CHANNEL_IDX_YAW]      - 1500.0f) / 500.0f;
    float throt = ((float)s_Guidance.rc.channels[RC_CHANNEL_IDX_THROTTLE] - 1000.0f) / 1000.0f;

    /*
     * Roll and pitch command an ANGLE, yaw commands a rate. The outer P
     * loop below turns angle error into the rate setpoint the rate PIDs in
     * control.c already close on, so centring the stick means LEVEL rather
     * than "stop rotating" - which is what a rate command means, and what
     * used to leave a permanent bank behind every stick input.
     *
     * Yaw is deliberately still a rate: there is no attitude to level to,
     * heading is wherever the pilot last left it.
     *
     * Units: nav euler and the gains are DEGREES, sp.w is rad/s, hence the
     * conversion on the way out. The clamp is the same full-scale rate the
     * stick used to command directly, so the rate loop below sees nothing
     * outside the range it was tuned for.
     */
    if ((nav.valid & NAV_VALID_ATTITUDE) != 0U) {
        float rollCmdDeg  = roll  * CFG_ANGLE_MAX_DEG;
        float pitchCmdDeg = pitch * CFG_ANGLE_MAX_DEG;

        /* Bound the pair as a VECTOR, not per axis. Clipping each one
         * separately rotates the commanded direction - the same argument
         * Mixer_MixMotors makes for the motor pair - and on the diagonal
         * the two sticks together reach 42 deg, which is not what
         * CFG_ANGLE_MAX_DEG says the limit is. */
        float magDeg = sqrtf ((rollCmdDeg * rollCmdDeg) + (pitchCmdDeg * pitchCmdDeg));
        if (magDeg > CFG_ANGLE_MAX_DEG) {
            float scale = CFG_ANGLE_MAX_DEG / magDeg;
            rollCmdDeg  *= scale;
            pitchCmdDeg *= scale;
        }

        float rollErrDeg  = rollCmdDeg  - nav.euler[0];
        float pitchErrDeg = pitchCmdDeg - nav.euler[1];

        sp.w[0] = clipf32 (DEG2RAD (CFG_ANGLE_ROLL_P * rollErrDeg),
                           -GUIDANCE_MAX_RATE_RAD_S, GUIDANCE_MAX_RATE_RAD_S);
        sp.w[1] = clipf32 (DEG2RAD (CFG_ANGLE_PITCH_P * pitchErrDeg),
                           -GUIDANCE_MAX_RATE_RAD_S, GUIDANCE_MAX_RATE_RAD_S);
    } else {
        /* No attitude estimate, so there is no error to close on. Fall back
         * to the rate command the stick used to mean: worse to fly, but it
         * needs no estimate, where holding a bank against a stale euler
         * angle would command one the vehicle is not in. */
        sp.w[0] = roll  * GUIDANCE_MAX_RATE_RAD_S;
        sp.w[1] = pitch * GUIDANCE_MAX_RATE_RAD_S;
    }
    sp.w[2] = yaw * GUIDANCE_MAX_RATE_RAD_S;

    sp.vel_b[0] = 0.0f;
    sp.vel_b[1] = 0.0f;

    if (altHold) {
        /*
         * vel_b[2] carries a TARGET ALTITUDE in metres here, which control
         * closes a PID around. In the passthrough fallback below it is instead
         * the raw normalised throttle the mixer consumes directly - same field,
         * two meanings, chosen by whether the altitude estimate exists.
         */
        float alt = -nav.pos_ned[2];   // NED, down positive

        /* Capture on entry so the mode holds where the vehicle IS, not
         * wherever the target was left by a previous engagement.
         *
         * Disarmed, RE-capture it every iteration and do not let the stick
         * walk it. With altitude hold always on (mission.c) the pre-arm hold
         * sits with the throttle stick at minimum, which is a full-scale
         * DESCENT command - left to run it walks the target metres below the
         * ground before the vehicle has even armed, and the first thing the
         * loop does on arming is chase it down. */
        if (!s_Guidance.altHoldActive || (s_Guidance.mission.armed == 0U)) {
            s_Guidance.altTarget     = alt;
            s_Guidance.altHoldActive = true;
        }

        /* Stick commands a climb RATE: centre holds, and the target only
         * moves while the pilot is asking for it. */
        float climbCmd = (throt - 0.5f) * 2.0f;   // -1..+1 about centre
        if ((climbCmd > -CFG_ALT_HOLD_STICK_DEADBAND) &&
            (climbCmd < CFG_ALT_HOLD_STICK_DEADBAND)) {
            climbCmd = 0.0f;
        }

        /*
         * climb_rate is published as exactly d(altTarget)/dt, and control.c's
         * vertical damping subtracts it from the measured climb rate. That
         * makes it a contract, not just telemetry: it must stay zero on any
         * iteration where the target is NOT walking, or the damping term
         * becomes a feedforward for motion the target is not making and the
         * loop chases it. Hence it is computed inside the same armed test that
         * gates the integration below rather than from climbCmd directly.
         */
        float climbRate = 0.0f;
        if (s_Guidance.mission.armed != 0U) {
            climbRate = climbCmd * CFG_ALT_HOLD_CLIMB_RATE_MPS;
            s_Guidance.altTarget += climbRate * dt;
        }

        sp.vel_b[2]   = s_Guidance.altTarget;
        sp.climb_rate = climbRate;
    } else {
        sp.vel_b[2] = throt;
        /* Passthrough: vel_b[2] is a raw throttle, not an altitude, so there is
         * no target moving at any rate. */
        sp.climb_rate = 0.0f;
        s_Guidance.altHoldActive = false;
    }

    umsg_guidance_setpoints_publish(&sp);
    return eSTATUS_SUCCESS;
}

void Guidance_Task(void* args) {
    (void)args;
    Guidance_Init();
    while (1) {
        Guidance_Update();
    }
}
