#include <stdbool.h>
#include <stdint.h>

#include "scheduler.h"
#include "task.h"

#include "core/core.h"

#define MAX_NUM_TASKS 10U

FJ_DEFINE_SHARED(Task_t, m_SequentialTasks[][MAX_NUM_TASKS]) = {
 
    [CM7_IDX] = {
        { .taskFunction = Task_RcUpdate,        .taskName = "RC_Update",      .isEnabled = true },
        { .taskFunction = TaskImu_Update,        .taskName = "IMU_Update",      .isEnabled = true },
        { .taskFunction = TaskAttitudeUpdate,   .taskName = "Attitude_Update", .isEnabled = true },
        { .taskFunction = TaskPIDUpdate,        .taskName = "PID_Update",      .isEnabled = true },
        { .taskFunction = TaskMixerUpdate,      .taskName = "Mixer_Update",    .isEnabled = true },
    },

    [CM4_IDX] = {
        { .taskFunction = Task_RxUpdate, .taskName = "Task_RxUpdate", .isEnabled = true },
        // { TaskCommandProcess,   "Command_Process",   0, 0, 300, true,  true },
        // { TaskSafetyCheck,      "Safety_Check",      0, 0, 100, true,  true },
        // { TaskStatusUpdate,     "Status_Update",     0, 0, 150, false, true },
    }
};

FJ_DEFINE_SHARED(Task_t, m_AsyncTasks[][MAX_NUM_TASKS]) = {

    [CM7_IDX] = {
        { .taskFunction = TaskInterCoreSync, .taskName = "InterCore_Sync", .isEnabled = true },
        // for now let main core handle logging flight data
        // because if not the data will need be synchronized
        { .taskFunction = Task_LogFlightData,      .taskName = "Log_FlightData", .hzUpdate = 10,    .isEnabled = true },
        { .taskFunction = Task_LogHeartBeat, .taskName = "Log_HeartBeat", .hzUpdate = 10, .isEnabled = true },
    },

    [CM4_IDX] = {
        { .taskFunction = TaskInterCoreSync, .taskName = "InterCore_Sync", .isEnabled = true },
        { .taskFunction = Task_LogHeartBeat, .taskName = "Log_HeartBeat", .hzUpdate = 10, .isEnabled = true },
    }
};

static uint32_t ExecuteTask (Task_t* pTask, uint32_t usCurrentTime, uint32_t usDeltaTime) {

    uint32_t usTaskStart    = GetMicroseconds ();
    eSTATUS_t status        = pTask->taskFunction (usTaskStart, usDeltaTime);
    uint32_t usTaskDuration = GetMicroseconds () - usTaskStart;
    if (STATUS_FAIL (status)) {
        LOG_ERROR ("Task %s failed with status %d", pTask->taskName, status);
    }

    pTask->usLastUpdateTime        = usTaskStart;
    pTask->usExpectedExecutionTime = usTaskDuration;
    pTask->usExecutionTimeSum += usTaskDuration;
    if (++pTask->nExecutions >= 10U) {
        pTask->usExpectedExecutionTime = pTask->usExecutionTimeSum / pTask->nExecutions;
        pTask->usExecutionTimeSum      = 0;
        pTask->nExecutions             = 0;
    }
    return usTaskDuration;
}

void Scheduler_Main (uint32_t coreIdx, uint32_t loopRateHz) {

    uint32_t usTotalLoopTime        = 1000000 / loopRateHz;
    Task_t (*pSequ)[MAX_NUM_TASKS]  = &m_SequentialTasks[coreIdx];
    Task_t (*pAsync)[MAX_NUM_TASKS] = &m_AsyncTasks[coreIdx];
    while (true) {

        uint32_t usLoopStart = GetMicroseconds ();
        uint32_t usRemaining = usTotalLoopTime;
        for (uint32_t i = 0; i < MAX_NUM_TASKS; ++i) {

            Task_t* pTask = pSequ[i];
            if (!pTask->isEnabled) {
                continue;
            }

            uint32_t usTaskDuration = ExecuteTask (pTask, usLoopStart, usLoopStart - pTask->usLastUpdateTime);
            if (usTaskDuration > usRemaining) {
                LOG_WARN ("Main loop exceeded time budget during task %s", pTask->taskName);
                usRemaining = 0;
                break;
            }
            usRemaining -= usTaskDuration;
        }

        if (usRemaining == 0) {
            continue;
        }

        for (uint32_t i = 0; i < MAX_NUM_TASKS; ++i) {

            Task_t* pTask = pAsync[i];
            if (!pTask->isEnabled || (usLoopStart - pTask->usLastUpdateTime) < HZ_TO_US (pTask->hzUpdate)) {
                continue;
            }

            uint32_t usTaskDuration = ExecuteTask (pTask, usLoopStart, usLoopStart - pTask->usLastUpdateTime);
            if (usTaskDuration > usRemaining) {
                LOG_WARN ("Async Task %s exceeded time budget", pTask->taskName);
                usRemaining = 0;
                break;
            }
            usRemaining -= usTaskDuration;
        }

        DelayMicroseconds (usRemaining);
    }
}