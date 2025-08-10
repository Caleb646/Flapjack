#ifndef SYNC_SYNC_H
#define SYNC_SYNC_H

#include "common.h"
#include "mem/mem.h"
#include <stdint.h>

#define MAILBOX_CM4_ID (0U)
#define MAILBOX_CM7_ID (1U)

typedef uint8_t eSYNC_TASKID_t;
enum {
    eSYNC_TASKID_UART_OUT = 0U,

    NUMBER_OF_SYNC_TASKS,
    eSYNC_TASKID_MAX = 256U
};

typedef struct {
    uint64_t padding;
} DefaultTask;

typedef struct {
    uint8_t unused;
    uint8_t taskID;
    uint16_t magic;
} SyncTaskHeader;

typedef struct {
    SyncTaskHeader header;
    uint16_t len;
} SyncTaskUartOut;

STATIC_ASSERT (sizeof (DefaultTask) == MEM_SHARED_MAILBOX_LEN, "");
STATIC_ASSERT (sizeof (SyncTaskHeader) == 4U, "");
STATIC_ASSERT (sizeof (SyncTaskUartOut) <= MEM_SHARED_MAILBOX_LEN, "");

typedef eSTATUS_t (*task_handler_fn_t) (DefaultTask const* pTask);

#ifdef UNIT_TEST

eSTATUS_t SyncMailBoxWrite (uint32_t mbID, uint8_t const* pBuffer, uint32_t len);
eSTATUS_t SyncMailBoxWriteNotify (uint32_t mbID, uint8_t const* pBuffer, uint32_t len);
eSTATUS_t SyncMailBoxRead (uint32_t mbID, uint8_t* pBuffer, uint32_t len);
uint16_t SyncGetOtherCoresMailBoxID (void);
uint8_t SyncTaskIsValid (DefaultTask const* pTask);
task_handler_fn_t SyncGetTaskHandler (uint32_t taskID);
void SyncIRQHandler (uint16_t myCPUMailBoxId);

#endif

eSTATUS_t SyncInit (void);
eSTATUS_t SyncRegisterHandler (eSYNC_TASKID_t taskID, task_handler_fn_t);
eSTATUS_t SyncProcessTasks (void);
eSTATUS_t SyncNotifyTaskUartOut (uint16_t len);

#endif // SYNC_SYNC_H