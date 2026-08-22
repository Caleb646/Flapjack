#include "tasks/mission/mission.h"

#include "drivers/rx/rx.h"

#include "umsg_rc.h"
#include "umsg_mission.h"
#include "umsg_arming.h"
#include "umsg_nav.h"

#include "FreeRTOS.h"

#define ARM_AUX_THRESHOLD     1750U
#define DISARM_AUX_THRESHOLD  1250U
/* Single threshold, not the two-sided hysteresis the arm switch uses: a
 * mode change is not an edge-triggered latch, so there is no state to get
 * stuck in if the stick sits near the boundary. */
#define ARM_THROTTLE_MAX      1100U
#define MISSION_RC_TIMEOUT_MS 20U

/*
 * Attitude-settle interlock. The Madgwick estimate needs a few seconds to pull
 * in from its identity-quaternion start, and NAV_VALID_ATTITUDE is set from the
 * first sample onwards - it says "the filter is running", not "the filter has
 * converged". Arming on that alone hands the rate loop a setpoint error of tens
 * of degrees and the aircraft departs immediately.
 *
 * So require the estimate to have actually stopped moving: every Euler axis
 * within ARM_ATTITUDE_STABLE_DEG of a reference for ARM_ATTITUDE_SETTLE_US. Any
 * axis leaving the band re-arms the reference and restarts the clock, which
 * also blocks arming while the airframe is being handled.
 */
#define ARM_ATTITUDE_STABLE_DEG 1.0F
#define ARM_ATTITUDE_SETTLE_US  3000000U

typedef enum {
    ARM_INTENT_NONE = 0,
    ARM_INTENT_ARM,
    ARM_INTENT_DISARM,
} ArmIntent_t;

typedef struct {
    umsg_sub_handle_t   rc_sub;
    umsg_sub_handle_t   arm_req_sub;
    umsg_sub_handle_t   nav_sub;
    umsg_mission_mode_t mode;
    bool                isArmed;
    bool                haveRc;
    bool                haveNav;
    bool                rcArmedRegion;
    bool                rcArmPending;
    umsg_rc_input_t     lastRc;
    umsg_nav_state_t    lastNav;
    // Attitude-settle tracking (see ARM_ATTITUDE_* above).
    bool                haveSettleRef;
    float               settleEuler[3];
    uint32_t            usSettleSince;
    char const*         pLastBlocker;
} Mission_t;

static Mission_t s_Mission;

// NaN and +/-Inf without pulling in math.h: NaN fails self-comparison, and the
// magnitude test rejects the infinities that would hang the wrap loop below.
//
// -Wfloat-equal is suppressed rather than satisfied: `v == v` IS the test, not a
// sloppy equality that wants an epsilon. Comparing v against a tolerance would
// not detect NaN at all, since every comparison with NaN is false. The
// suppression is scoped to this function so the flag keeps working everywhere
// else.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wfloat-equal"
static bool Mission_IsFinite(float v) {
    return (v == v) && (v <= 3.0e38f) && (v >= -3.0e38f);
}
#pragma GCC diagnostic pop

// Absolute angular difference in degrees, wrap-safe so a yaw estimate crossing
// +/-180 does not read as 360 degrees of movement.
static float Mission_AngleDeltaDeg(float a, float b) {
    float d = a - b;
    while (d > 180.0f)  { d -= 360.0f; }
    while (d < -180.0f) { d += 360.0f; }
    return d < 0.0f ? -d : d;
}

// Fold the latest nav estimate into the settle tracker. Restarts the window
// whenever any axis leaves the band, or whenever the estimate is not finite.
static void Mission_TrackAttitudeSettle(void) {

    if (!s_Mission.haveNav || !(s_Mission.lastNav.valid & NAV_VALID_ATTITUDE)) {
        s_Mission.haveSettleRef = false;
        return;
    }

    float const* pEuler = s_Mission.lastNav.euler;
    for (uint32_t i = 0; i < 3U; ++i) {
        if (!Mission_IsFinite(pEuler[i])) {
            s_Mission.haveSettleRef = false;
            return;
        }
    }

    uint32_t usNow = GetMicroseconds();

    bool restart = !s_Mission.haveSettleRef;
    for (uint32_t i = 0; i < 3U && !restart; ++i) {
        if (Mission_AngleDeltaDeg(pEuler[i], s_Mission.settleEuler[i]) > ARM_ATTITUDE_STABLE_DEG) {
            restart = true;
        }
    }

    if (restart) {
        for (uint32_t i = 0; i < 3U; ++i) {
            s_Mission.settleEuler[i] = pEuler[i];
        }
        s_Mission.usSettleSince = usNow;
        s_Mission.haveSettleRef = true;
    }
}

static bool Mission_IsAttitudeSettled(void) {
    return s_Mission.haveSettleRef &&
           (GetMicroseconds() - s_Mission.usSettleSince) >= ARM_ATTITUDE_SETTLE_US;
}

eSTATUS_t Mission_Init(void) {
    s_Mission.rc_sub        = umsg_rc_input_subscribe(1, 4);
    s_Mission.arm_req_sub   = umsg_arming_request_subscribe(1, 4);
    s_Mission.nav_sub       = umsg_nav_state_subscribe(1, 1);
    s_Mission.mode          = EMISSION_MODE_ALTITUDE_HOLD;
    s_Mission.isArmed       = false;
    s_Mission.haveRc        = false;
    s_Mission.haveNav       = false;
    s_Mission.rcArmedRegion = false;
    s_Mission.rcArmPending  = false;
    s_Mission.haveSettleRef = false;
    s_Mission.pLastBlocker  = NULL;
    return eSTATUS_SUCCESS;
}

// Arming preconditions. Returns NULL when armable, else why not - the strings
// are literals, so the caller can compare pointers to log only on change.
static char const* Mission_ArmBlocker(void) {

    if (!s_Mission.haveNav || !(s_Mission.lastNav.valid & NAV_VALID_ATTITUDE)) {
        return "no valid attitude estimate";
    }
    if (!Mission_IsAttitudeSettled()) {
        return "attitude estimate still settling";
    }
    if (s_Mission.haveRc && s_Mission.lastRc.channels[RC_CHANNEL_IDX_THROTTLE] > ARM_THROTTLE_MAX) {
        return "throttle not at minimum";
    }
    return NULL;
}

eSTATUS_t Mission_Update(void) {

    // Block on RC with a timeout so mission stays responsive (and can arm via shell) with no RC.
    umsg_rc_input_t rc;
    if (umsg_rc_input_receive(s_Mission.rc_sub, &rc, pdMS_TO_TICKS(MISSION_RC_TIMEOUT_MS))) {
        s_Mission.lastRc = rc;
        s_Mission.haveRc = true;
    }

    // Latest nav state; cached so armability does not depend on a nav message
    // landing in this exact iteration.
    umsg_nav_state_t nav;
    if (umsg_nav_state_receive(s_Mission.nav_sub, &nav, 0)) {
        s_Mission.lastNav = nav;
        s_Mission.haveNav = true;
    }
    Mission_TrackAttitudeSettle();

    /* One flight mode: hold altitude on the throttle stick, hold attitude on
     * roll and pitch. There is no AUX2 mode switch and no manual alternative -
     * a pilot should not have to pick, and every mode boundary is a transient
     * to get wrong.
     *
     * This is not the same as having no fallback. Both loops degrade on their
     * own inputs: guidance drops to throttle passthrough without
     * NAV_VALID_BARO_ALT, and to a rate command without NAV_VALID_ATTITUDE.
     * Those are sensor failures, not modes. */
    s_Mission.mode = EMISSION_MODE_ALTITUDE_HOLD;

    // RC arm-switch intent (edge-detected with hysteresis).
    ArmIntent_t rcIntent = ARM_INTENT_NONE;
    if (s_Mission.haveRc) {
        uint32_t aux = s_Mission.lastRc.channels[RC_CHANNEL_IDX_AUX_1];
        if (aux > ARM_AUX_THRESHOLD) {
            if (!s_Mission.rcArmedRegion) { rcIntent = ARM_INTENT_ARM; }
            s_Mission.rcArmedRegion = true;
        } else if (aux < DISARM_AUX_THRESHOLD) {
            if (s_Mission.rcArmedRegion) { rcIntent = ARM_INTENT_DISARM; }
            s_Mission.rcArmedRegion = false;
        }
    }

    // Shell arm request (event).
    ArmIntent_t shellIntent = ARM_INTENT_NONE;
    umsg_arming_request_t req;
    if (umsg_arming_request_receive(s_Mission.arm_req_sub, &req, 0)) {
        shellIntent = req.arm ? ARM_INTENT_ARM : ARM_INTENT_DISARM;
    }

    // A raised arm switch is a standing request, not a one-shot edge: the
    // attitude gate can take seconds to open, and an edge-only request would be
    // dropped and never retried while the pilot holds the switch up. The
    // request is cleared the moment the switch leaves the armed region.
    if (rcIntent == ARM_INTENT_ARM) {
        s_Mission.rcArmPending = true;
    }
    if (!s_Mission.rcArmedRegion) {
        s_Mission.rcArmPending = false;
    }

    // Resolve: a disarm from either source wins; otherwise arm if armable.
    // A shell request stays one-shot - it is an explicit command, so a rejection
    // is reported and dropped rather than latched.
    if (rcIntent == ARM_INTENT_DISARM || shellIntent == ARM_INTENT_DISARM) {
        if (s_Mission.isArmed) { LOG_INFO("Mission: disarmed"); }
        s_Mission.isArmed       = false;
        s_Mission.rcArmPending  = false;
        s_Mission.pLastBlocker  = NULL;
    } else if (s_Mission.rcArmPending || shellIntent == ARM_INTENT_ARM) {
        char const* pBlocker = s_Mission.isArmed ? NULL : Mission_ArmBlocker();
        if (pBlocker == NULL) {
            if (!s_Mission.isArmed) { LOG_INFO("Mission: armed"); }
            s_Mission.isArmed      = true;
            s_Mission.rcArmPending = false;
            s_Mission.pLastBlocker = NULL;
        } else if (pBlocker != s_Mission.pLastBlocker) {
            // Literals, so a pointer compare logs once per reason rather than
            // once per iteration while the request waits.
            LOG_WARN("Mission: arm blocked - %s", pBlocker);
            s_Mission.pLastBlocker = pBlocker;
        }
    }

    umsg_mission_state_t msg;
    msg.mode           = s_Mission.mode;
    msg.target_pos[0]  = 0.0f;
    msg.target_pos[1]  = 0.0f;
    msg.target_pos[2]  = 0.0f;
    msg.target_heading = 0.0f;
    msg.armed          = s_Mission.isArmed ? 1U : 0U;
    umsg_mission_state_publish(&msg);
    return eSTATUS_SUCCESS;
}

void Mission_Task(void* args) {
    (void)args;
    Mission_Init();
    while (1) {
        Mission_Update();
    }
}
