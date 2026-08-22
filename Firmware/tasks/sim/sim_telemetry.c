#include "tasks/sim/sim_telemetry.h"

#include "core/core.h"

#include "drivers/sim_link/sim_link.h"

#include "drivers/rx/rx.h"

#include "umsg_nav.h"
#include "umsg_mission.h"
#include "umsg_sensors.h"

#include "FreeRTOS.h"
#include "task.h"

/*
 * Deep enough that a drain per 50 Hz iteration never misses a publish: baro
 * runs at the bridge's baro rate (~50 Hz) against this task's 50 Hz, so with a
 * depth of 1 ordinary jitter would silently drop samples and under-report the
 * count. The counts below are the pacing metric - they have to be exact.
 */
#define SIMTLM_SENSOR_QUEUE_DEPTH 8U

eSTATUS_t SimTelemetry_Init (void) {
    return SimLink_Init();
}

void SimTelemetry_Task (void* args) {

    (void)args;

    /* Depth 1 is right here, unlike the sensor queues below: imu_status carries
     * a running total, so the newest message is the whole answer and a missed
     * one costs nothing. */
    umsg_sub_handle_t imu_sub     = umsg_sensors_imu_status_subscribe (1, 1);
    umsg_sub_handle_t nav_sub     = umsg_nav_state_subscribe (1, 1);
    umsg_sub_handle_t mission_sub = umsg_mission_state_subscribe (1, 1);
    umsg_sub_handle_t baro_sub    = umsg_sensors_baro_subscribe (1, SIMTLM_SENSOR_QUEUE_DEPTH);
    umsg_sub_handle_t gps_sub     = umsg_sensors_gps_subscribe (1, SIMTLM_SENSOR_QUEUE_DEPTH);

    // Latest-value caches: receive() consumes, so hold the last value for
    // iterations where nothing new arrived.
    umsg_sensors_imu_status_t imuStatus = { 0 };
    umsg_nav_state_t     nav     = { 0 };
    umsg_mission_state_t mission = { 0 };
    umsg_sensors_baro_t  baro    = { 0 };
    umsg_sensors_gps_t   gps     = { 0 };

    /*
     * Loopback counters. These count messages that reached the END of the
     * sensor chain (driver -> device -> task -> umsg publish), which is what
     * makes the bridge's round-trip assertion meaningful: a parser that decodes
     * a frame and then discards its fields moves SimLink's own frame counter but
     * never moves this one.
     */
    uint32_t baroCount = 0;
    uint32_t gpsCount  = 0;

    for (;;) {
        umsg_sensors_imu_status_receive (imu_sub, &imuStatus, 0);
        umsg_nav_state_receive (nav_sub, &nav, 0);
        umsg_mission_state_receive (mission_sub, &mission, 0);

        // Drain rather than take-one: an exact count, whatever the rate ratio.
        umsg_sensors_baro_t baroRx;
        while (umsg_sensors_baro_receive (baro_sub, &baroRx, 0)) {
            baro = baroRx;
            baroCount++;
        }
        umsg_sensors_gps_t gpsRx;
        while (umsg_sensors_gps_receive (gps_sub, &gpsRx, 0)) {
            gps = gpsRx;
            gpsCount++;
        }

        /* Element-wise rather than a memcpy after the fact: C cannot copy one
         * array into another inside a designated initializer, and splitting the
         * copies out below would mean a forgotten one leaves the field silently
         * zeroed by the initializer. Spelled out here, every field is visible in
         * one place and the compiler checks the counts. */
        SimLinkTelemetry_t tlm = {
            .eulerDeg  = { nav.euler[0], nav.euler[1], nav.euler[2] },
            .armed     = mission.armed != 0U,
            .imuCount  = imuStatus.sample_count,
            .rcLinkUp  = Rx_IsLinkUp (),
            .baroPa    = baro.pressure,
            .baroCount = baroCount,
            .gpsLat    = gps.lat,
            .gpsLon    = gps.lon,
            .gpsAlt    = gps.alt,
            .gpsSats   = gps.sats,
            .gpsCount  = gpsCount,
            .posNed    = { nav.pos_ned[0], nav.pos_ned[1], nav.pos_ned[2] },
            .velNed    = { nav.vel_ned[0], nav.vel_ned[1], nav.vel_ned[2] },
            .navValid  = nav.valid,
        };
        SimLink_SendTelemetry (&tlm);
        vTaskDelay (pdMS_TO_TICKS (20));   // 50 Hz
    }
}
