#include "mission/mission.h"
#include "mc/motors.h"
#include "fc/rc.h"
#include "umsg_rc.h"
#include "umsg_mission.h"

#include "FreeRTOS.h"

#define MISSION_ARM_THRESHOLD    1750U
#define MISSION_DISARM_THRESHOLD 1250U

typedef struct {
    umsg_sub_handle_t rc_sub;
    bool isArmed;
    eMissionMode_t mode;
} Mission_t;

static Mission_t s_Mission;

eSTATUS_t Mission_Init(void) {
    s_Mission.rc_sub  = umsg_rc_input_subscribe(1, 4);
    s_Mission.isArmed = false;
    s_Mission.mode    = eMISSION_MODE_MANUAL;
    return eSTATUS_SUCCESS;
}

eSTATUS_t Mission_Update(void) {
    umsg_rc_input_t rc;
    if (!umsg_rc_input_receive(s_Mission.rc_sub, &rc, portMAX_DELAY)) {
        return eSTATUS_FAILURE;
    }

    uint32_t aux1 = rc.channels[RC_CHANNEL_IDX_AUX_1];

    if (aux1 > MISSION_ARM_THRESHOLD && !s_Mission.isArmed) {
        if (STATUS_OK(Motors_Arm())) {
            s_Mission.isArmed = true;
            LOG_INFO("Mission: armed");
        }
    } else if (aux1 < MISSION_DISARM_THRESHOLD && s_Mission.isArmed) {
        Motors_Disarm();
        s_Mission.isArmed = false;
        LOG_INFO("Mission: disarmed");
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
