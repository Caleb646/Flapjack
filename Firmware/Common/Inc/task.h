#ifndef TASK_H
#define TASK_H

#include <stdbool.h>
#include <stdint.h>

#include "core/core.h"

typedef struct {
    eSTATUS_t (*taskFunction) (uint32_t usCurrentTime, uint32_t usDeltaTime);
    char const* taskName;
    uint32_t hzUpdate;
    uint32_t usLastUpdateTime;
    uint32_t usExpectedExecutionTime;
    uint32_t usExecutionTimeSum;
    uint32_t nExecutions;
    bool isEnabled;
} Task_t;

eSTATUS_t TaskMixerUpdate (uint32_t usCurrentTime, uint32_t usDeltaTime);
eSTATUS_t TaskPIDUpdate (uint32_t usCurrentTime, uint32_t usDeltaTime);
eSTATUS_t TaskAttitudeUpdate (uint32_t usCurrentTime, uint32_t usDeltaTime);
eSTATUS_t TaskImu_Update (uint32_t usCurrentTime, uint32_t usDeltaTime);
eSTATUS_t TaskMag_Update (uint32_t usCurrentTime, uint32_t usDeltaTime);
eSTATUS_t TaskInterCoreSync (uint32_t usCurrentTime, uint32_t usDeltaTime);
eSTATUS_t Task_LogHeartBeat (uint32_t usCurrentTime, uint32_t usDeltaTime);
eSTATUS_t Task_LogFlightData (uint32_t usCurrentTime, uint32_t usDeltaTime);
eSTATUS_t Task_RxUpdate (uint32_t usCurrentTime, uint32_t usDeltaTime);
eSTATUS_t Task_RcUpdate (uint32_t usCurrentTime, uint32_t usDeltaTime);


#endif /* TASK_H */