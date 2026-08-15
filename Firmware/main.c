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

#define STACK_SENSOR   128U
#define STACK_NAV      512U
#define STACK_GUIDANCE 256U
#define STACK_CONTROL  512U
#define STACK_MISSION  128U
#define STACK_RX       128U

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
