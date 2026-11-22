#include "tasks/heartbeat/heartbeat.h"

#include "core/core.h"

#include "umsg_control.h"
#include "umsg_guidance.h"
#include "umsg_mission.h"
#include "umsg_nav.h"
#include "umsg_rc.h"
#include "umsg_sensors.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdint.h>

/*
 * One status line every 5 s, so a quiet debug UART can be told apart from a
 * hung FC - the firmware otherwise logs only on events, and goes silent once
 * it has booted.
 *
 * Liveness comes from topic publish RATES, not from the tasks reporting on
 * themselves: every task that matters publishes once per loop, so a task that
 * stalled, or suspended itself after a failed init, reads 0 Hz here. The beat
 * itself arriving only proves the scheduler is not saturated, since this task
 * runs at the lowest priority.
 */
#define HEARTBEAT_PERIOD_MS 5000U

typedef enum {
    RATE_IMU,
    RATE_MAG,
    RATE_BARO,
    RATE_GPS,
    RATE_RC,
    RATE_NAV,
    RATE_GUIDANCE,
    RATE_MISSION,
    RATE_CONTROL,
    RATE_COUNT
} eRate_t;

static uint32_t (*const s_TopicCount[RATE_COUNT]) (void) = {
    [RATE_IMU]      = umsg_sensors_imu_count,
    [RATE_MAG]      = umsg_sensors_mag_count,
    [RATE_BARO]     = umsg_sensors_baro_count,
    [RATE_GPS]      = umsg_sensors_gps_count,
    [RATE_RC]       = umsg_rc_input_count,
    [RATE_NAV]      = umsg_nav_state_count,
    [RATE_GUIDANCE] = umsg_guidance_setpoints_count,
    [RATE_MISSION]  = umsg_mission_state_count,
    [RATE_CONTROL]  = umsg_control_status_count,
};

static char const* const s_MotorStateName[] = {
    [CONTROL_MOTORS_DISARMED] = "disarmed",
    [CONTROL_MOTORS_ARMING]   = "arming",
    [CONTROL_MOTORS_ARMED]    = "armed",
};

/* Static rather than on the stack: the message copies alone are ~200 B. */
typedef struct {
    umsg_sub_handle_t     navSub;
    umsg_sub_handle_t     missionSub;
    umsg_sub_handle_t     gpsSub;
    umsg_sub_handle_t     rcSub;
    umsg_sub_handle_t     controlSub;
    umsg_nav_state_t      nav;
    umsg_mission_state_t  mission;
    umsg_sensors_gps_t    gps;
    umsg_rc_input_t       rc;
    umsg_control_status_t control;
    uint32_t              prevCount[RATE_COUNT];
    uint32_t              prevMotorWrites;
    uint32_t              prevServoWrites;
    uint32_t              usPrev;
} Heartbeat_t;

static Heartbeat_t s_Hb;

/* Per second over the time that actually passed: at the lowest priority a
 * beat can land late, and dividing by the nominal period would over-read. */
static unsigned int PerSecond (uint32_t delta, uint32_t usElapsed) {
    return (unsigned int)(((uint64_t)delta * 1000000ULL) / usElapsed);
}

void Heartbeat_Task (void* args) {

    (void)args;

    s_Hb.navSub     = umsg_nav_state_subscribe (1, 1);
    s_Hb.missionSub = umsg_mission_state_subscribe (1, 1);
    s_Hb.gpsSub     = umsg_sensors_gps_subscribe (1, 1);
    s_Hb.rcSub      = umsg_rc_input_subscribe (1, 1);
    s_Hb.controlSub = umsg_control_status_subscribe (1, 1);

    for (uint32_t i = 0; i < RATE_COUNT; ++i) {
        s_Hb.prevCount[i] = s_TopicCount[i]();
    }
    s_Hb.usPrev = GetMicroseconds ();

    TickType_t lastWake = xTaskGetTickCount ();
    while (1) {
        xTaskDelayUntil (&lastWake, pdMS_TO_TICKS (HEARTBEAT_PERIOD_MS));

        uint32_t const usNow     = GetMicroseconds ();
        uint32_t const usElapsed = usNow - s_Hb.usPrev;
        s_Hb.usPrev              = usNow;

        unsigned int hz[RATE_COUNT];
        for (uint32_t i = 0; i < RATE_COUNT; ++i) {
            uint32_t const count = s_TopicCount[i]();
            hz[i]                = PerSecond (count - s_Hb.prevCount[i], usElapsed);
            s_Hb.prevCount[i]    = count;
        }

        /* Timeout 0: no new message keeps the last one, and the 0 Hz above
         * already says it is stale. */
        (void)umsg_nav_state_receive (s_Hb.navSub, &s_Hb.nav, 0);
        (void)umsg_mission_state_receive (s_Hb.missionSub, &s_Hb.mission, 0);
        (void)umsg_sensors_gps_receive (s_Hb.gpsSub, &s_Hb.gps, 0);
        (void)umsg_rc_input_receive (s_Hb.rcSub, &s_Hb.rc, 0);
        (void)umsg_control_status_receive (s_Hb.controlSub, &s_Hb.control, 0);

        unsigned int const motorWr = PerSecond (s_Hb.control.motor_writes - s_Hb.prevMotorWrites, usElapsed);
        unsigned int const servoWr = PerSecond (s_Hb.control.servo_writes - s_Hb.prevServoWrites, usElapsed);
        s_Hb.prevMotorWrites       = s_Hb.control.motor_writes;
        s_Hb.prevServoWrites       = s_Hb.control.servo_writes;

        LOG_INFO (
        "HB up=%us heap=%u armed=%u mode=%u nav_valid=0x%x | hz imu=%u mag=%u baro=%u gps=%u "
        "rc=%u nav=%u guid=%u mis=%u ctl=%u | gps fix=%u sats=%u | rc lq=%u | ctl init=0x%x "
        "motors=%s mot_wr=%u srv_wr=%u",
        (unsigned int)(xTaskGetTickCount () / configTICK_RATE_HZ),
        (unsigned int)xPortGetFreeHeapSize (),
        (unsigned int)s_Hb.mission.armed,
        (unsigned int)s_Hb.mission.mode,
        (unsigned int)s_Hb.nav.valid,
        hz[RATE_IMU],
        hz[RATE_MAG],
        hz[RATE_BARO],
        hz[RATE_GPS],
        hz[RATE_RC],
        hz[RATE_NAV],
        hz[RATE_GUIDANCE],
        hz[RATE_MISSION],
        hz[RATE_CONTROL],
        (unsigned int)s_Hb.gps.fix_type,
        (unsigned int)s_Hb.gps.sats,
        (unsigned int)s_Hb.rc.link_quality,
        (unsigned int)s_Hb.control.init_ok,
        s_MotorStateName[s_Hb.control.motor_state],
        motorWr,
        servoWr);
    }
}
