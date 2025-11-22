#ifndef SYNC_SYNC_H
#define SYNC_SYNC_H

#include "core/core_shared.h"
#include <stdint.h>

#define MAILBOX_CM4_ID (0U)
#define MAILBOX_CM7_ID (1U)
#define MAILBOX_COUNT  (2U)

typedef uint8_t eSYNC_TASKID_t;
enum {
    eSYNC_TASKID_UART_OUT = 0U,

    eSYNC_TASKID_MAX
};

typedef struct MailBox_t {
    uint8_t data[8U];
} MailBox_t;

// typedef MailBox_t volatile vMailBox_t;
typedef MailBox_t vMailBox_t;

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

FJ_STATIC_ASSERT (sizeof (SyncTaskHeader) == 4U, "");

FJ_STATIC_ASSERT (sizeof (SyncTaskUartOut) < sizeof (MailBox_t), "");

typedef eSTATUS_t (*task_handler_fn_t) (DefaultTask const* pTask);

#ifdef UNIT_TEST

eSTATUS_t SyncMailBoxWrite (uint32_t mbID, uint8_t const* pBuffer, uint32_t len);
eSTATUS_t SyncMailBoxWriteNotify (uint32_t mbID, uint8_t const* pBuffer, uint32_t len);
eSTATUS_t SyncMailBoxRead (uint32_t mbID, uint8_t* pBuffer, uint32_t len);
uint16_t SyncGetOtherCoresMailBoxID (void);
task_handler_fn_t SyncGetTaskHandler (eSYNC_TASKID_t taskID);
void SyncIRQHandler (uint16_t myCPUMailBoxId);

#endif

eSTATUS_t SyncInit (void);
eSTATUS_t SyncRegisterHandler (eSYNC_TASKID_t taskID, task_handler_fn_t);
eSTATUS_t SyncProcessTasks (void);
eSTATUS_t SyncNotifyTaskUartOut (uint16_t len);

bool LockTake (void);
void LockRelease (void);
#endif // SYNC_SYNC_H