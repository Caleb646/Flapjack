#include "control/control.h"
#include "mc/mixer.h"
#include "mc/motors.h"
#include "mc/pid.h"
#include "mc/servos.h"
#include "umsg_guidance.h"
#include "umsg_mission.h"
#include "umsg_nav.h"

#include "FreeRTOS.h"

#include <string.h>

// Max rate used to normalise PID output to [-1, 1] for the mixer.
// Matches the maximum rate guidance can command (π rad/s → 180 deg/s).
#define CONTROL_MAX_RATE_DEG_S 180.0f

typedef struct {
    umsg_sub_handle_t guidance_sub;
    uint32_t          usLastUpdateTime;
} Control_t;

static Control_t s_Control;

eSTATUS_t Control_Init(void) {
    s_Control.guidance_sub      = umsg_guidance_setpoints_subscribe(1, 4);
    s_Control.usLastUpdateTime  = GetMicroseconds();
    return eSTATUS_SUCCESS;
}

eSTATUS_t Control_Update(void) {
    umsg_guidance_setpoints_t sp;
    if (!umsg_guidance_setpoints_receive(s_Control.guidance_sub, &sp, portMAX_DELAY)) {
        return eSTATUS_FAILURE;
    }

    umsg_nav_state_t nav;
    umsg_nav_state_peek(&nav);

    umsg_mission_state_t mission;
    umsg_mission_state_peek(&mission);

    uint32_t usNow = GetMicroseconds();
    float dt = (float)(usNow - s_Control.usLastUpdateTime) / 1000000.0f;
    s_Control.usLastUpdateTime = usNow;

    // Rate PID: guidance w[] is in rad/s, nav gyro[] is in deg/s from the IMU.
    // Convert targets to deg/s to match the sensor units.
    float pid_data[AXIS_IDX_COUNT];
    float targets[3] = {
        RAD2DEG(sp.w[0]),
        RAD2DEG(sp.w[1]),
        RAD2DEG(sp.w[2]),
    };

    Pid_t* pPid = Pid_Get();
    for (uint32_t i = 0; i < 3U; ++i) {
        float output = Pid_UpdateAxis_(&pPid->axes[i], nav.gyro[i], targets[i], dt);
        pid_data[i] = clipf32(output, -CONTROL_MAX_RATE_DEG_S, CONTROL_MAX_RATE_DEG_S)
                      / CONTROL_MAX_RATE_DEG_S;
    }
    pid_data[AXIS_IDX_THROTTLE] = sp.vel_b[2];

    Mixer_MixMotors(&g_Mixer, pid_data, g_Mixer.motorOutputs);
    memset(g_Mixer.servoOutputs, 0, sizeof(g_Mixer.servoOutputs));
    Mixer_MixServos(&g_Mixer, pid_data, g_Mixer.servoOutputs);

    if (!mission.armed) {
        return eSTATUS_SUCCESS;
    }

    eSTATUS_t status = Servos_Write(g_Mixer.servoOutputs);
    if (STATUS_FAIL(status)) {
        LOG_ERROR("Control: failed to write servo outputs");
        return status;
    }

    status = Motors_Write(g_Mixer.motorOutputs);
    if (STATUS_FAIL(status)) {
        LOG_ERROR("Control: failed to write motor outputs");
        return status;
    }

    return eSTATUS_SUCCESS;
}
