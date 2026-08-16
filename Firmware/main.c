#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "hal.h"
#include "platform.h"
#include "target.h"

#include "core/core.h"

#include "FreeRTOS.h"
#include "task.h"

#include "devices/serial.h"

#include "tasks/imu/imu_task.h"
#include "tasks/mag/mag_task.h"
#include "tasks/rc/rc.h"
#include "tasks/nav/nav.h"
#include "tasks/guidance/guidance.h"
#include "tasks/control/control.h"
#include "tasks/mission/mission.h"
#include "tasks/rx/rx_task.h"
#include "tasks/shell/shell.h"

#include "drivers/sim_link/sim_link.h"
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
#define TASK_PRIORITY_SENSOR_RC  3U
#define TASK_PRIORITY_NAV        5U
#define TASK_PRIORITY_GUIDANCE   4U
#define TASK_PRIORITY_CONTROL    5U
#define TASK_PRIORITY_MISSION    3U
#define TASK_PRIORITY_RX         3U

#define TASK_PRIORITY_SIMLINK 6U
#define TASK_PRIORITY_SIMTLM  1U

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
#define STACK_SENSOR   256U   /* imu was at 77% of 128 - one LOG_ERROR from overflow */
#define STACK_NAV      512U   /* 42% - fine */
#define STACK_GUIDANCE 384U
#define STACK_CONTROL  512U   /* 49% - fine */
#define STACK_MISSION  384U
#define STACK_RX       128U   /* CM4-only; not exercised by the single-core SIL, so unmeasured */

#define STACK_SIMLINK         512U
#define STACK_SIMTLM          256U

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

#ifdef SIM_HIL
    /* HIL: the debug UART becomes the binary sim link; logging is suppressed and
     * the shell is unavailable. */
    if (STATUS_FAIL (SimLink_Init ())) {
        CriticalErrorHandler ();
    }
#else
    if (STATUS_FAIL (SerialDebug_Init ())) {
        CriticalErrorHandler ();
    }
#endif

#if !defined(SINGLE_CORE)
    s_IsSystemInited = true;
    while (!s_IsCM4Ready) {
        // allow cm4 to initialize logger
    };
#endif /* !SINGLE_CORE */

#ifndef SIM_HIL
    if (STATUS_FAIL (Shell_Init ())) {
        LOG_ERROR ("Failed to init shell");
        CriticalErrorHandler ();
    }
#endif

    LOG_INFO ("Starting scheduler");
    xTaskCreate (Imu_Task,      "imu",      STACK_SENSOR,   NULL, TASK_PRIORITY_SENSOR_IMU, NULL);
    xTaskCreate (Mag_Task,      "mag",      STACK_SENSOR,   NULL, TASK_PRIORITY_SENSOR_MAG, NULL);
    xTaskCreate (Rc_Task,       "rc",       STACK_SENSOR,   NULL, TASK_PRIORITY_SENSOR_RC,  NULL);
    xTaskCreate (Nav_Task,      "nav",      STACK_NAV,      NULL, TASK_PRIORITY_NAV,        NULL);
    xTaskCreate (Guidance_Task, "guidance", STACK_GUIDANCE, NULL, TASK_PRIORITY_GUIDANCE,   NULL);
    xTaskCreate (Control_Task,  "control",  STACK_CONTROL,  NULL, TASK_PRIORITY_CONTROL,    NULL);
    xTaskCreate (Mission_Task,  "mission",  STACK_MISSION,  NULL, TASK_PRIORITY_MISSION,    NULL);
#ifdef SIM_HIL
    xTaskCreate (SimLink_RxTask,    "simrx",  STACK_SIMLINK, NULL, TASK_PRIORITY_SIMLINK, NULL);
    xTaskCreate (SimTelemetry_Task, "simtlm", STACK_SIMTLM,  NULL, TASK_PRIORITY_SIMTLM,  NULL);
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
    xTaskCreate (Rx_Task, "rx", STACK_RX, NULL, TASK_PRIORITY_RX, NULL);
    vTaskStartScheduler ();

#endif /* CORE_CM4 */
    while (1);
}
