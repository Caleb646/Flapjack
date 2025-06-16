#ifndef SYNC_SYNC_H
#define SYNC_SYNC_H

#include "common.h"
#include "stdint.h"

#define SYNC_TASKID_UART_OUT ((uint32_t)0)

// typedef struct {
//     uint32_t taskID;
//     uint8_t padding_[64 - sizeof(taskID)];
// } DefaultSyncTask;

// STATIC_ASSERT(sizeof(DefaultSyncTask) == 64, "");

typedef STATUS_TYPE (*task_handler_fn_t) ();

STATUS_TYPE SyncInit (void);
STATUS_TYPE SyncRegisterHandler (task_handler_fn_t, uint32_t);
task_handler_fn_t SyncGetTaskHandler (uint32_t taskID);

// typedef struct {
//     uint32_t taskID;
//     void *pOutBuf;
//     uint32_t len;
// } SyncTaskUARTOut;

// STATIC_ASSERT(sizeof(SyncTaskUARTOut) <= 64, "");


#endif // SYNC_SYNC_H