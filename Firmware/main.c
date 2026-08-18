#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "hal.h"
#include "platform.h"
#include "target.h"

#include "core/core.h"

#include "FreeRTOS.h"
#include "task.h"

#include "drivers/bus/spi.h"
#include "drivers/serial/serial_link.h"
#include "drivers/serial/uart.h"

#include "tasks/imu/imu_task.h"
#include "tasks/mag/mag_task.h"
#include "tasks/baro/baro_task.h"
#include "tasks/gps/gps_task.h"
#include "tasks/rc/rc.h"
#include "tasks/nav/nav.h"
#include "tasks/guidance/guidance.h"
#include "tasks/control/control.h"
#include "tasks/mission/mission.h"
#include "tasks/shell/shell.h"
#include "tasks/serial_link/serial_link.h"
#include "tasks/sim/sim_telemetry.h"

/*
 * Stack-overflow trap (configCHECK_FOR_STACK_OVERFLOW == 2).
 *
 * By the time this runs the overflowing task has already scribbled past its
 * stack, so do NOT log from here - the logger is printf-shaped and wants the
 * very stack that just ran out. Park the task name somewhere a debugger can
 * read it and halt.
 */
char const* volatile g_pOverflowedTaskName = NULL;

void vApplicationStackOverflowHook (TaskHandle_t xTask, char* pcTaskName);
void vApplicationStackOverflowHook (TaskHandle_t xTask, char* pcTaskName) {

    FJ_UNUSED (xTask);
    g_pOverflowedTaskName = pcTaskName;
    CriticalErrorHandler ();
}

FJ_DEFINE_SHARED (bool volatile, s_IsCM4Stuck)     = false;
FJ_DEFINE_SHARED (bool volatile, s_IsSystemInited) = false;
FJ_DEFINE_SHARED (bool volatile, s_IsCM4Ready)     = false;

/* ── FreeRTOS task resourcing ───────────────────────────────────────────────
 * main.c is the composition root: every task's XX_Task() lives in its own task
 * file; main() registers them per core below.
 */

#define TASK_PRIORITY_SENSOR_IMU 2U
#define TASK_PRIORITY_SENSOR_MAG 1U
/* Baro sits with mag at the bottom of the sensor group: it is a ~50 Hz part and
 * nothing consumes it on the flight path yet, so it must never preempt the
 * 400 Hz imu chain. */
#define TASK_PRIORITY_SENSOR_BARO 1U
/* GPS polls a UART-fed buffer at 100 Hz and nothing on the flight path consumes
 * it yet. One above baro/mag so a poll is not deferred behind them long enough
 * to miss a sentence - the driver only holds one. */
#define TASK_PRIORITY_SENSOR_GPS  2U
#define TASK_PRIORITY_SENSOR_RC  3U
#define TASK_PRIORITY_NAV        5U
#define TASK_PRIORITY_GUIDANCE   4U
#define TASK_PRIORITY_CONTROL    5U
#define TASK_PRIORITY_MISSION    3U

#define TASK_PRIORITY_SERIAL_RX 6U
#define TASK_PRIORITY_SIMTLM    1U
/*
 * The serial TX task sits at the lowest application priority deliberately. A
 * sustained log flood keeps it permanently runnable, and anywhere above 1 that
 * would starve mag and simtlm - the fault recorded as KnownIssues 2.6. At 1 it
 * time-slices with them instead, and the link degrades by dropping text.
 */
#define TASK_PRIORITY_SERIAL_TX 1U

/*
 * Stack depths are in WORDS (x4 for bytes) and are sized from measurement, not
 * guesswork: uxTaskGetStackHighWaterMark() was sampled for every task over a
 * SIL run. Peak usage observed, against the old allocation:
 *
 *     imu 99   mag 82   rc 82   mission 158   guidance 142
 *     control 253   nav 213   simrx 236   idle 24
 *
 * The mission figure is the smoking gun for the overflow that once wedged the
 * sim link with no diagnostic at all: 158 words used against 128 allocated. Its
 * locals are a umsg_rc_input_t (68 B) plus a umsg_nav_state_t (72 B) plus the
 * arming request and outgoing state - ~170 B of message structs before any call
 * frame - and it logs, which is printf-shaped and stack-hungry.
 *
 * Anything that can log is now given roughly 2x its measured peak, because a
 * log call is the largest and rarest frame each task takes. configCHECK_FOR_
 * STACK_OVERFLOW is on (see the hook above), so a regression halts loudly.
 */
#define STACK_SENSOR   256U   /* imu was at 77% of 128 - one LOG_ERROR from overflow.
                               * rc also carries the CRSF decode and its link
                               * up/down logging: measured 129 words peak, 50%. */
#define STACK_NAV      512U   /* 42% - fine */
#define STACK_GUIDANCE 384U
#define STACK_CONTROL  512U   /* 49% - fine */
#define STACK_MISSION  384U
/*
 * Measured over a SIL run with uxTaskGetStackHighWaterMark(), same method as
 * the table above: slrx peaked at 285 words, sltx at 51.
 *
 * sltx keeps 256 rather than being trimmed to its 51-word peak because that
 * measurement only covers the single-core path, where SyncProcessTasks() finds
 * an empty queue and returns immediately. In a dual-core build it runs the
 * logger's sink handler, and that path is deeper and unmeasured.
 */
#define STACK_SERIAL_RX       512U   /* 285 used - the shell decode and its logs land here now */
#define STACK_SERIAL_TX       256U   /* 51 used single-core; must never log (see serial_link.c) */
#define STACK_SIMTLM          256U

/*
 * Every task below is required for flight, so a failed creation must not be
 * allowed to pass quietly - the vehicle would boot into a scheduler missing,
 * say, the control loop. xTaskCreate only fails by returning
 * errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY, i.e. the heap ran out.
 *
 * LOG_ERROR is safe here: the scheduler is not running yet, so SerialLink
 * bypasses its buffering and writes straight to the UART. At CFG_LOG_LEVEL
 * none the message compiles out, but the halt still happens.
 */
#define CREATE_TASK(fn, name, stack, prio)                                       \
    do {                                                                         \
        if (xTaskCreate ((fn), (name), (stack), NULL, (prio), NULL) != pdPASS) { \
            LOG_ERROR ("Failed to create task: %s", (name));                     \
            CriticalErrorHandler ();                                             \
        }                                                                        \
    } while (0)

int main (void) {

    if (Platform_Init () != 0) {
        CriticalErrorHandler ();
    }

#if defined(CORE_CM7)

    if (STATUS_FAIL (Spi_InitSystem ())) {
        CriticalErrorHandler ();
    }

    if (STATUS_FAIL (Uart_InitSystem ())) {
        CriticalErrorHandler ();
    }

    if (STATUS_FAIL (Core_Init ())) {
        CriticalErrorHandler ();
    }

    /* Sole owner of the debug UART. Log text and binary frames share it, so
     * logging and the shell stay available in a sim build. */
    if (STATUS_FAIL (SerialLink_Init ())) {
        CriticalErrorHandler ();
    }

#if !defined(SINGLE_CORE)
    s_IsSystemInited = true;
    while (!s_IsCM4Ready) {
        // allow cm4 to initialize logger
    };
#endif /* !SINGLE_CORE */

#ifdef SIM_HIL
    if (STATUS_FAIL (SimTelemetry_Init ())) {
        LOG_ERROR ("Failed to init sim telemetry");
        CriticalErrorHandler ();
    }
#endif

    if (STATUS_FAIL (Shell_Init ())) {
        LOG_ERROR ("Failed to init shell");
        CriticalErrorHandler ();
    }

    LOG_INFO ("Starting scheduler");
    CREATE_TASK (Imu_Task,          "imu",      STACK_SENSOR,    TASK_PRIORITY_SENSOR_IMU);
    CREATE_TASK (Mag_Task,          "mag",      STACK_SENSOR,    TASK_PRIORITY_SENSOR_MAG);
    CREATE_TASK (Baro_Task,         "baro",     STACK_SENSOR,    TASK_PRIORITY_SENSOR_BARO);
    CREATE_TASK (Gps_Task,          "gps",      STACK_SENSOR,    TASK_PRIORITY_SENSOR_GPS);
    CREATE_TASK (Rc_Task,           "rc",       STACK_SENSOR,    TASK_PRIORITY_SENSOR_RC);
    CREATE_TASK (Nav_Task,          "nav",      STACK_NAV,       TASK_PRIORITY_NAV);
    CREATE_TASK (Guidance_Task,     "guidance", STACK_GUIDANCE,  TASK_PRIORITY_GUIDANCE);
    CREATE_TASK (Control_Task,      "control",  STACK_CONTROL,   TASK_PRIORITY_CONTROL);
    CREATE_TASK (Mission_Task,      "mission",  STACK_MISSION,   TASK_PRIORITY_MISSION);
    CREATE_TASK (SerialLink_RxTask, "slrx",     STACK_SERIAL_RX, TASK_PRIORITY_SERIAL_RX);
    CREATE_TASK (SerialLink_TxTask, "sltx",     STACK_SERIAL_TX, TASK_PRIORITY_SERIAL_TX);
#ifdef SIM_HIL
    CREATE_TASK (SimTelemetry_Task, "simtlm",   STACK_SIMTLM,    TASK_PRIORITY_SIMTLM);
#endif

    LOG_INFO ("Heap Free Size: %u", (uint16_t)xPortGetFreeHeapSize ());
    vTaskStartScheduler ();
#endif /* CORE_CM7 */

/*
 * ******************* CM4 Start *******************
 */
#if defined(CORE_CM4)

    while (!s_IsSystemInited) {
        // wait for cm7 to init system
    };

    if (Core_Init () != eSTATUS_SUCCESS) {
        s_IsCM4Stuck = true;
        CriticalErrorHandler ();
    }

    s_IsCM4Ready = true;
    LOG_INFO ("Starting scheduler");
    /*
     * No application tasks. The receiver used to live here and reach CM7 through
     * shared memory; it is now part of Rc_Task on CM7, which removed the last
     * cross-core data flow. The scheduler still starts so the core has an idle
     * task and the logger's sync path keeps working.
     *
     * CM4 is idle capacity. Note that whatever lands here next cannot publish a
     * umsg topic to a CM7 subscriber - umsg is plain FreeRTOS queues with
     * per-core metadata, so it does not cross cores. Use core/sync.c for that.
     */
    vTaskStartScheduler ();

#endif /* CORE_CM4 */
    while (1);
}
