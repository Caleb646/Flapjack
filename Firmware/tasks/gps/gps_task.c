#include "tasks/gps/gps_task.h"

#include "devices/gps.h"

#include "drivers/device.h"
#include "drivers/gps/gpsdrv.h"

#include "umsg_sensors.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdbool.h>

/*
 * Fallback timeout, not the clock: the UART RX ISR signals every completed
 * sentence, so the notification normally lands first. It stays at the old poll
 * period so a receiver that is silent, or a build that passes no signal,
 * degrades to exactly the previous polling behaviour.
 *
 * It must not become portMAX_DELAY, and the reason is stronger here than on the
 * mag and baro tasks. The fix-loss branch below exists for a receiver that has
 * stopped talking - and one that has stopped talking raises no interrupt, so
 * only this timeout can wake the task to notice. Blocking forever would leave a
 * dead antenna looking like a good fix indefinitely.
 *
 * Waking per sentence also closes the window the old 100 Hz poll left open: the
 * driver holds exactly ONE assembled sentence (s_SentenceBuffer in
 * drivers/gps/gps.c), and a second landing before this task consumes the first
 * overwrites it. That window is now this task's scheduling latency instead of a
 * full poll period.
 */
#define GPS_POLL_PERIOD_MS 10U

/*
 * NVIC priority for the GPS UART. Gps_DataReady runs at it and calls
 * vTaskNotifyGiveFromISR, so it must sit at or below the kernel's syscall
 * ceiling - numerically >=, since lower numbers preempt. Asserted here because
 * this is the layer that knows: drivers/ takes the number on faith to avoid
 * pulling FreeRTOSConfig.h down there.
 */
#define GPS_RX_IRQ_PRIORITY 5U

STATIC_ASSERT (GPS_RX_IRQ_PRIORITY >= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY,
               "GPS RX ISR must not preempt the FreeRTOS kernel");

static Gps_t s_Gps;

/* ISR context, once per assembled sentence. Signal only - the parse happens
 * back in the task. */
static void Gps_DataReady (void* ctx) {

    BaseType_t higherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR ((TaskHandle_t)ctx, &higherPriorityTaskWoken);
    portYIELD_FROM_ISR (higherPriorityTaskWoken);
}

void Gps_Task (void* args) {

    (void)args;

    /*
     * The notification target is this task, so Gps_Init has to run here rather
     * than from whoever created it - a task notification has no existence apart
     * from the task that receives it.
     */
    DataReadySignal_t signal = {
        .Notify      = Gps_DataReady,
        .ctx         = xTaskGetCurrentTaskHandle (),
        .irqPriority = GPS_RX_IRQ_PRIORITY,
    };

    if (STATUS_FAIL (Gps_Init (&s_Gps, &signal))) {
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

        (void)ulTaskNotifyTake (pdTRUE, pdMS_TO_TICKS (GPS_POLL_PERIOD_MS));

        /*
         * Publish only on SUCCESS, which the driver returns only for a sentence
         * that actually carried a position. Everything else is ordinary traffic,
         * NOT an error, and must not be logged: the 10 ms fallback timeout wakes
         * this task far more often than a 10 Hz receiver emits sentences, so most
         * iterations are still eSTATUS_BUSY (nothing assembled yet), and the
         * non-positional sentence types plus a receiver reporting no lock account
         * for most of the rest. Logging any of those would put ~100 lines/second
         * onto the shared debug UART - the log-flood failure KnownIssues 2.6
         * records as starving the low-priority tasks.
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
    }
}
