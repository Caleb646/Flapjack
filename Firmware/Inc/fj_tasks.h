#ifndef FJ_TASKS_H
#define FJ_TASKS_H

#include "common.h"

typedef struct Task_s Task_t;

typedef struct Task_s {
    eSTATUS_t (*fn) (Task_t* pSelf, uint32_t currentTimeUs);
    void* pArg;
    char const* pName;
} Task_t;

FJ_DECLARE_SHARED (Task_t, e_PrimaryTasks[]);
FJ_DECLARE_SHARED (uint8_t, e_nPrimaryTasks);

FJ_DECLARE_SHARED (Task_t, e_SecondaryTasks[]);
FJ_DECLARE_SHARED (uint8_t, e_nSecondaryTasks);

eSTATUS_t Task_UpdateMain (Task_t* pSelf, uint32_t currentTimeUs);
eSTATUS_t Task_UpdateSecondary (Task_t* pSelf, uint32_t currentTimeUs);
eSTATUS_t Task_UpdateAcc (Task_t* pSelf, uint32_t currentTimeUs);
eSTATUS_t Task_UpdateGyro (Task_t* pSelf, uint32_t currentTimeUs);
eSTATUS_t Task_UpdateMag (Task_t* pSelf, uint32_t currentTimeUs);
eSTATUS_t Task_UpdateBaro (Task_t* pSelf, uint32_t currentTimeUs);
eSTATUS_t Task_UpdateGps (Task_t* pSelf, uint32_t currentTimeUs);
eSTATUS_t Task_UpdateAttitude (Task_t* pSelf, uint32_t currentTimeUs);
eSTATUS_t Task_UpdateAltitude (Task_t* pSelf, uint32_t currentTimeUs);
eSTATUS_t Task_UpdatePid (Task_t* pSelf, uint32_t currentTimeUs);
eSTATUS_t Task_UpdateMixer (Task_t* pSelf, uint32_t currentTimeUs);


#endif // FJ_TASKS_H