#include "tasks/guidance/guidance.h"
#include "target.h"
#include "core/core.h"
#include "drivers/rx/rx.h"
#include "umsg_nav.h"
#include "umsg_mission.h"
#include "umsg_rc.h"
#include "umsg_guidance.h"

#include "FreeRTOS.h"

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

    /* Zero-initialised because the branch below does not cover every mode.
     * Publishing an uninitialised local would put stack garbage on the rate and
     * throttle setpoints, and the mixer forwards whatever it is given straight
     * to the actuators. */
    umsg_guidance_setpoints_t sp = { 0 };
    sp.quat[0] = 1.0f;
    sp.quat[1] = 0.0f;
    sp.quat[2] = 0.0f;
    sp.quat[3] = 0.0f;

    /*
     * Gated on NAV_VALID_BARO_ALT as well as the mode: without a usable altitude
     * there is nothing to capture a target from, so fall back to the manual
     * throttle passthrough. A centred stick there is 1500 us -> 0.5, which is
     * hover thrust, so the fallback holds rather than drops.
     */
    bool altHold = (s_Guidance.mission.mode == EMISSION_MODE_ALTITUDE_HOLD) &&
                   ((nav.valid & NAV_VALID_BARO_ALT) != 0U);

    if ((s_Guidance.mission.mode == EMISSION_MODE_MANUAL) ||
        (s_Guidance.mission.mode == EMISSION_MODE_ALTITUDE_HOLD)) {
        float roll  = ((float)s_Guidance.rc.channels[RC_CHANNEL_IDX_ROLL]     - 1500.0f) / 500.0f;
        float pitch = ((float)s_Guidance.rc.channels[RC_CHANNEL_IDX_PITCH]    - 1500.0f) / 500.0f;
        float yaw   = ((float)s_Guidance.rc.channels[RC_CHANNEL_IDX_YAW]      - 1500.0f) / 500.0f;
        float throt = ((float)s_Guidance.rc.channels[RC_CHANNEL_IDX_THROTTLE] - 1000.0f) / 1000.0f;

        // Both modes fly attitude the same way; only throttle differs.
        sp.w[0] = roll  * GUIDANCE_MAX_RATE_RAD_S;
        sp.w[1] = pitch * GUIDANCE_MAX_RATE_RAD_S;
        sp.w[2] = yaw   * GUIDANCE_MAX_RATE_RAD_S;

        sp.vel_b[0] = 0.0f;
        sp.vel_b[1] = 0.0f;

        if (altHold) {
            /*
             * ALTITUDE_HOLD reinterprets vel_b[2] as a TARGET ALTITUDE in
             * metres, which control closes a PID around; in MANUAL it stays the
             * raw normalised throttle the mixer consumes directly.
             */
            float alt = -nav.pos_ned[2];   // NED, down positive

            /* Capture on entry so the mode holds where the vehicle IS, not
             * wherever the target was left by a previous engagement. */
            if (!s_Guidance.altHoldActive) {
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
            s_Guidance.altTarget += climbCmd * CFG_ALT_HOLD_CLIMB_RATE_MPS * dt;

            sp.vel_b[2] = s_Guidance.altTarget;
        } else {
            sp.vel_b[2] = throt;
            s_Guidance.altHoldActive = false;
        }
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
