#include "tasks/guidance/guidance.h"
#include "drivers/rx/rx.h"
#include "umsg_nav.h"
#include "umsg_mission.h"
#include "umsg_rc.h"
#include "umsg_guidance.h"

#include "FreeRTOS.h"

#define GUIDANCE_MAX_RATE_RAD_S 3.14159265f

typedef struct {
    umsg_sub_handle_t nav_sub;
} Guidance_t;

static Guidance_t s_Guidance;

eSTATUS_t Guidance_Init(void) {
    s_Guidance.nav_sub = umsg_nav_state_subscribe(1, 4);
    return eSTATUS_SUCCESS;
}

eSTATUS_t Guidance_Update(void) {
    umsg_nav_state_t nav;
    if (!umsg_nav_state_receive(s_Guidance.nav_sub, &nav, portMAX_DELAY)) {
        return eSTATUS_FAILURE;
    }

    umsg_mission_state_t mission;
    umsg_mission_state_peek(&mission);

    umsg_rc_input_t rc;
    umsg_rc_input_peek(&rc);

    umsg_guidance_setpoints_t sp;
    sp.quat[0] = 1.0f;
    sp.quat[1] = 0.0f;
    sp.quat[2] = 0.0f;
    sp.quat[3] = 0.0f;

    if (mission.mode == EMISSION_MODE_MANUAL) {
        float roll  = ((float)rc.channels[RC_CHANNEL_IDX_ROLL]     - 1500.0f) / 500.0f;
        float pitch = ((float)rc.channels[RC_CHANNEL_IDX_PITCH]    - 1500.0f) / 500.0f;
        float yaw   = ((float)rc.channels[RC_CHANNEL_IDX_YAW]      - 1500.0f) / 500.0f;
        float throt = ((float)rc.channels[RC_CHANNEL_IDX_THROTTLE] - 1000.0f) / 1000.0f;

        sp.w[0] = roll  * GUIDANCE_MAX_RATE_RAD_S;
        sp.w[1] = pitch * GUIDANCE_MAX_RATE_RAD_S;
        sp.w[2] = yaw   * GUIDANCE_MAX_RATE_RAD_S;

        sp.vel_b[0] = 0.0f;
        sp.vel_b[1] = 0.0f;
        sp.vel_b[2] = throt;
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
