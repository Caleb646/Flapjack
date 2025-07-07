#ifndef SYNC_SYNC_H
#define SYNC_SYNC_H

#include "common.h"
#include <stdint.h>

#define MAILBOX_CM4_ID (0U)
#define MAILBOX_CM7_ID (1U)

typedef enum {
    eSYNC_TASKID_UART_OUT = 0U,


    eSYNC_TASKID_MAX = 256U
} eSYNC_TASKID_TYPE;

typedef struct {
    union {
        uint32_t unused;
        uint8_t taskID;
        uint16_t magic;
    };
} SyncTaskHeader;

typedef struct {
    SyncTaskHeader header;
    uint16_t len;
} SyncTaskUartOut;

STATIC_ASSERT (sizeof (SyncTaskUartOut) <= 8U, "");

typedef STATUS_TYPE (*task_handler_fn_t) ();

STATUS_TYPE SyncInit (void);
STATUS_TYPE SyncRegisterHandler (eSYNC_TASKID_TYPE taskID, task_handler_fn_t);

#ifdef UNIT_TEST
void SyncTaskQueue_Init (SyncTaskQueue* q);
int SyncTaskQueue_IsEmpty (const SyncTaskQueue* q);
int SyncTaskQueue_IsFull (const SyncTaskQueue* q);
int SyncTaskQueue_Enqueue (SyncTaskQueue* q, const SyncTaskHeader* task);
int SyncTaskQueue_Dequeue (SyncTaskQueue* q, SyncTaskHeader* out_task);
int SyncTaskQueue_Peek (const SyncTaskQueue* q, SyncTaskHeader* out_task);

STATUS_TYPE SyncMailBoxWrite (uint32_t mbID, uint8_t* pBuffer, uint32_t len);
STATUS_TYPE SyncMailBoxWriteNotify (uint32_t mbID, uint8_t* pBuffer, uint32_t len);
STATUS_TYPE SyncMailBoxRead (uint32_t mbID, uint8_t* pBuffer, uint32_t len);

task_handler_fn_t SyncGetTaskHandler (uint32_t taskID);
#endif

// typedef struct {
//     uint32_t taskID;
//     void *pOutBuf;
//     uint32_t len;
// } SyncTaskUARTOut;

// STATIC_ASSERT(sizeof(SyncTaskUARTOut) <= 64, "");


#endif // SYNC_SYNC_H