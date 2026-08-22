#include "tasks/gps/gps_task.h"

#include "devices/gps.h"
#include "drivers/gps/gpsdrv.h"

#include "umsg_sensors.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdbool.h>

/*
 * Polled, not blocking - the same shape as Rc_Task, and for the same reason:
 * sentences arrive on a UART ISR with no notification path to wait on, so most
 * polls find nothing ready and that is the ordinary case rather than an error.
 *
 * 100 Hz rather than the RC path's 50 Hz because the driver holds exactly ONE
 * assembled sentence (s_SentenceBuffer + s_IsSentenceReady in drivers/gps/gps.c).
 * A second sentence landing before this task consumes the first overwrites it,
 * so the poll has to outpace the sentence rate rather than merely match the fix
 * rate: a 10 Hz receiver emitting GGA + RMC is 20 sentences/s. The SIL bridge
 * spaces the two halves of a fix by half a period for the same reason.
 */
#define GPS_POLL_PERIOD_MS 10U

static Gps_t s_Gps;

void Gps_Task (void* args) {

    (void)args;

    if (STATUS_FAIL (Gps_Init (&s_Gps))) {
        LOG_ERROR ("GPS unavailable; task exiting");
        /* Not vTaskDelete: this build uses heap_1, whose vPortFree asserts -
         * and configASSERT spins with interrupts disabled, wedging the FC. */
        vTaskSuspend (NULL);
        return;
    }

    /* Latches the last published validity, so the loss of a fix is announced
     * exactly once rather than at the 100 Hz poll rate. Starts false so a board
     * that never gets a fix stays silent instead of reporting a loss at boot. */
    bool hadFix = false;

    while (true) {
        /*
         * Publish only on SUCCESS, which the driver returns only for a sentence
         * that actually carried a position. Everything else is ordinary traffic,
         * NOT an error, and must not be logged: at a 100 Hz poll against a 10 Hz
         * receiver, ~98% of iterations are eSTATUS_BUSY (no sentence assembled
         * yet), and the non-positional sentence types plus a receiver reporting
         * no lock account for most of the rest. Logging any of those would put
         * ~100 lines/second onto the shared debug UART - the log-flood failure
         * KnownIssues 2.6 records as starving the low-priority tasks.
         *
         * Rc_Task documents the same reasoning for the same reason.
         */
        if (STATUS_OK (Gps_Update (&s_Gps))) {
            umsg_sensors_gps_t msg = {
                .lat      = s_Gps.data.latitude,
                .lon      = s_Gps.data.longitude,
                .alt      = s_Gps.data.altitude,
                .speed    = s_Gps.data.speed,
                .course   = s_Gps.data.course,
                .fix_type = s_Gps.data.fixType,
                .sats     = s_Gps.data.satellitesInUse,
            };
            umsg_sensors_gps_publish (&msg);
            hadFix = true;
        } else if (hadFix && !Gps_HasFix (&s_Gps)) {
            /*
             * The fix stopped being real - either the receiver reported loss of
             * lock, or it went silent and timed out. Say so on the topic rather
             * than simply going quiet: subscribers hold the last value they were
             * given, so silence leaves a stale fix looking current forever. This
             * is why Rc_Update publishes on its own clock too.
             *
             * Position and velocity are left at their last known values - a
             * return-to-home wants that point. Only the quality fields are
             * cleared, which is what marks the rest as not-current.
             */
            umsg_sensors_gps_t msg = {
                .lat      = s_Gps.data.latitude,
                .lon      = s_Gps.data.longitude,
                .alt      = s_Gps.data.altitude,
                .speed    = s_Gps.data.speed,
                .course   = s_Gps.data.course,
                .fix_type = 0U,
                .sats     = 0U,
            };
            umsg_sensors_gps_publish (&msg);
            hadFix = false;
            LOG_WARN ("GPS fix lost");
        }
        vTaskDelay (pdMS_TO_TICKS (GPS_POLL_PERIOD_MS));
    }
}
