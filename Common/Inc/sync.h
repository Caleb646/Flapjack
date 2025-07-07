#ifndef SYNC_SYNC_H
#define SYNC_SYNC_H

#include "common.h"
#include "mem/mem.h"
#include <stdint.h>

#define MAILBOX_CM4_ID (0U)
#define MAILBOX_CM7_ID (1U)

typedef enum {
    eSYNC_TASKID_UART_OUT = 0U,


    eSYNC_TASKID_MAX = 256U
} eSYNC_TASKID_TYPE;

typedef struct {
    uint64_t padding;
} DefaultTask;

STATIC_ASSERT (sizeof (DefaultTask) == MEM_SHARED_MAILBOX_LEN, "");

typedef struct {
    uint8_t unused;
    uint8_t taskID;
    uint16_t magic;
} SyncTaskHeader;

STATIC_ASSERT (sizeof (SyncTaskHeader) == 4U, "");

typedef struct {
    SyncTaskHeader header;
    uint16_t len;
} SyncTaskUartOut;


STATIC_ASSERT (sizeof (SyncTaskUartOut) <= MEM_SHARED_MAILBOX_LEN, "");

typedef STATUS_TYPE (*task_handler_fn_t) (DefaultTask const* pTask);

STATUS_TYPE SyncInit (void);
STATUS_TYPE SyncRegisterHandler (eSYNC_TASKID_TYPE taskID, task_handler_fn_t);
STATUS_TYPE SyncProcessTasks (void);
STATUS_TYPE SyncNotifyTaskUartOut (uint16_t len);


#ifdef UNIT_TEST

STATUS_TYPE SyncMailBoxWrite (uint32_t mbID, uint8_t* pBuffer, uint32_t len);
STATUS_TYPE SyncMailBoxWriteNotify (uint32_t mbID, uint8_t* pBuffer, uint32_t len);
STATUS_TYPE SyncMailBoxRead (uint32_t mbID, uint8_t* pBuffer, uint32_t len);
uint16_t SyncGetOtherCoresMailBoxID (void);
uint8_t SyncTaskIsValid (DefaultTask const* pTask);
task_handler_fn_t SyncGetTaskHandler (uint32_t taskID);

#endif

// typedef struct {
//     uint32_t taskID;
//     void *pOutBuf;
//     uint32_t len;
// } SyncTaskUARTOut;

// STATIC_ASSERT(sizeof(SyncTaskUARTOut) <= 64, "");


#endif // SYNC_SYNC_H