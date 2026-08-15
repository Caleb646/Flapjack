#include "tasks/mission/mission.h"

#include "drivers/rx/rx.h"

#include "umsg_rc.h"
#include "umsg_mission.h"
#include "umsg_arming.h"
#include "umsg_nav.h"

#include "FreeRTOS.h"

#define ARM_AUX_THRESHOLD     1750U
#define DISARM_AUX_THRESHOLD  1250U
#define ARM_THROTTLE_MAX      1100U
#define MISSION_RC_TIMEOUT_MS 20U

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
    umsg_rc_input_t     lastRc;
    umsg_nav_state_t    lastNav;
} Mission_t;

static Mission_t s_Mission;

eSTATUS_t Mission_Init(void) {
    s_Mission.rc_sub        = umsg_rc_input_subscribe(1, 4);
    s_Mission.arm_req_sub   = umsg_arming_request_subscribe(1, 4);
    s_Mission.nav_sub       = umsg_nav_state_subscribe(1, 1);
    s_Mission.mode          = EMISSION_MODE_MANUAL;
    s_Mission.isArmed       = false;
    s_Mission.haveRc        = false;
    s_Mission.haveNav       = false;
    s_Mission.rcArmedRegion = false;
    return eSTATUS_SUCCESS;
}

// Arming preconditions: attitude estimate valid, and (when RC is present) throttle at minimum.
static bool Mission_IsArmable(void) {
    bool navValid = s_Mission.haveNav && (s_Mission.lastNav.valid & NAV_VALID_ATTITUDE);

    bool throttleOk = true;
    if (s_Mission.haveRc) {
        throttleOk = s_Mission.lastRc.channels[RC_CHANNEL_IDX_THROTTLE] <= ARM_THROTTLE_MAX;
    }
    return navValid && throttleOk;
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

    // Resolve: a disarm from either source wins; otherwise the latest arm wins, only if armable.
    if (rcIntent == ARM_INTENT_DISARM || shellIntent == ARM_INTENT_DISARM) {
        if (s_Mission.isArmed) { LOG_INFO("Mission: disarmed"); }
        s_Mission.isArmed = false;
    } else if (rcIntent == ARM_INTENT_ARM || shellIntent == ARM_INTENT_ARM) {
        if (Mission_IsArmable()) {
            if (!s_Mission.isArmed) { LOG_INFO("Mission: armed"); }
            s_Mission.isArmed = true;
        } else {
            LOG_WARN("Mission: arm rejected (not armable)");
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
