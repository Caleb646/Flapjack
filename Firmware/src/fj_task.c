#include "fj_task.h"

#include "FreeRTOS.h"
#include "task.h"

#include "core/core.h"
#include "target.h"

#include "sensors/imu.h"
#include "sensors/mag.h"
#include "sensors/rc.h"

#include "nav/nav.h"
#include "guidance/guidance.h"
#include "control/control.h"
#include "mission/mission.h"

#include "drivers/rx/rx.h"

#define TASK_PRIORITY_SENSOR_IMU 2U
#define TASK_PRIORITY_SENSOR_MAG 1U
#define TASK_PRIORITY_SENSOR_RC  3U
#define TASK_PRIORITY_NAV        5U
#define TASK_PRIORITY_GUIDANCE   4U
#define TASK_PRIORITY_CONTROL    5U
#define TASK_PRIORITY_MISSION    3U
#define TASK_PRIORITY_RX         3U

#define STACK_SENSOR   128U
#define STACK_NAV      512U
#define STACK_GUIDANCE 256U
#define STACK_CONTROL  512U
#define STACK_MISSION  128U
#define STACK_RX       128U

static void SensorImu_Task(void* args) {
    (void)args;
    SensorImu_Init();
    while (1) {
        SensorImu_Update();
    }
}

static void SensorMag_Task(void* args) {
    (void)args;
    SensorMag_Init();
    while (1) {
        SensorMag_Update();
    }
}

static void SensorRc_Task(void* args) {
    (void)args;
    SensorRc_Init();
    while (1) {
        SensorRc_Update();
    }
}

static void Nav_Task(void* args) {
    (void)args;
    Nav_Init();
    while (1) {
        Nav_Update();
    }
}

static void Guidance_Task(void* args) {
    (void)args;
    Guidance_Init();
    while (1) {
        Guidance_Update();
    }
}

static void Control_Task(void* args) {
    (void)args;
    Control_Init();
    while (1) {
        Control_Update();
    }
}

static void Mission_Task(void* args) {
    (void)args;
    Mission_Init();
    while (1) {
        Mission_Update();
    }
}

static void Rx_Task(void* args) {
    (void)args;
    Rx_Init();
    uint32_t usLast = GetMicroseconds();
    while (1) {
        uint32_t usNow = GetMicroseconds();
        Rx_Update(usNow, usNow - usLast);
        usLast = usNow;
    }
}

void FjTasks_Start(uint32_t coreIdx) {

    if (coreIdx == CM7_IDX) {
        xTaskCreate(SensorImu_Task, "imu",      STACK_SENSOR,   NULL, TASK_PRIORITY_SENSOR_IMU, NULL);
        xTaskCreate(SensorMag_Task, "mag",      STACK_SENSOR,   NULL, TASK_PRIORITY_SENSOR_MAG, NULL);
        xTaskCreate(SensorRc_Task,  "rc",       STACK_SENSOR,   NULL, TASK_PRIORITY_SENSOR_RC,  NULL);
        xTaskCreate(Nav_Task,       "nav",      STACK_NAV,      NULL, TASK_PRIORITY_NAV,         NULL);
        xTaskCreate(Guidance_Task,  "guidance", STACK_GUIDANCE, NULL, TASK_PRIORITY_GUIDANCE,    NULL);
        xTaskCreate(Control_Task,   "control",  STACK_CONTROL,  NULL, TASK_PRIORITY_CONTROL,     NULL);
        xTaskCreate(Mission_Task,   "mission",  STACK_MISSION,  NULL, TASK_PRIORITY_MISSION,     NULL);
    } else {
        xTaskCreate(Rx_Task, "rx", STACK_RX, NULL, TASK_PRIORITY_RX, NULL);
    }

    vTaskStartScheduler();
}
