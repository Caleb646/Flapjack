#ifndef SYNC_H
#define SYNC_H

#include "stdint.h"

#include "common.h"

typedef enum {
    TASKID_PRINTF = 0
} TASKID;

typedef struct {
    uint32_t taskID;
} SyncTask;


#endif // SYNC_H