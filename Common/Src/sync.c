#include "sync.h"
#include "common.h"
#include "hal.h"
#include "mem/mem.h"
#include "mem/queue.h"
#include <string.h>


typedef enum {
    NUM_TASK_TYPES       = 2U,
    SYNC_TASK_QUEUE_SIZE = 128U,
    TASK_MAGIC           = 0xBEEF
} SyncConstants;

static DefaultTask gSyncTaskBuffer[SYNC_TASK_QUEUE_SIZE] = { 0 };
static Queue gSyncTaskQueue                              = { 0 };
static task_handler_fn_t gHandlers[NUM_TASK_TYPES]       = { 0 };

STATIC_TESTABLE_DECL uint8_t SyncTaskIsValid (DefaultTask const* pTask) {
    if (pTask == NULL) {
        return FALSE;
    }
    SyncTaskHeader const* pHeader = (SyncTaskHeader const*)pTask;
    // Check if the task ID is valid
    return pHeader->magic == TASK_MAGIC && pHeader->taskID < NUM_TASK_TYPES;
}

STATIC_TESTABLE_DECL uint16_t SyncGetOtherCoresMailBoxID (void) {
    if (HAL_GetCurrentCPUID () == CM7_CPUID) {
        return MAILBOX_CM4_ID;
    }
    return MAILBOX_CM7_ID;
}

STATIC_TESTABLE_DECL uint8_t volatile* SyncMailBoxGet (uint32_t mbID) {
    uint8_t volatile* pMB = NULL;
    if (mbID == MAILBOX_CM7_ID) {
        pMB = (uint8_t volatile*)MEM_SHARED_MAILBOX_CM7_START;
    } else {
        pMB = (uint8_t volatile*)MEM_SHARED_MAILBOX_CM4_START;
    }
    return pMB;
}

STATIC_TESTABLE_DECL STATUS_TYPE SyncMailBoxWrite (uint32_t mbID, uint8_t* pBuffer, uint32_t len) {
    if (len > MEM_SHARED_MAILBOX_LEN) {
        return eSTATUS_FAILURE;
    }

    uint8_t volatile* pMB = SyncMailBoxGet (mbID);
    memcpy ((void*)pMB, (void*)pBuffer, len);
    return eSTATUS_SUCCESS;
}

STATIC_TESTABLE_DECL STATUS_TYPE SyncMailBoxWriteNotify (uint32_t mbID, uint8_t* pBuffer, uint32_t len) {
    STATUS_TYPE status = SyncMailBoxWrite (mbID, pBuffer, len);
    if (status != eSTATUS_SUCCESS) {
        return eSTATUS_FAILURE;
    }
    asm volatile ("dsb");
    asm volatile ("sev");
    return eSTATUS_SUCCESS;
}

STATIC_TESTABLE_DECL STATUS_TYPE SyncMailBoxRead (uint32_t mbID, uint8_t* pBuffer, uint32_t len) {
    if (len > MEM_SHARED_MAILBOX_LEN) {
        return eSTATUS_FAILURE;
    }
    uint8_t volatile* pMB = SyncMailBoxGet (mbID);
    memcpy ((void*)pBuffer, (void*)pMB, len);
    return eSTATUS_SUCCESS;
}


STATIC_TESTABLE_DECL task_handler_fn_t SyncGetTaskHandler (uint32_t taskID) {
    if (taskID > NUM_TASK_TYPES) {
        return NULL;
    }
    return gHandlers[taskID];
}

STATIC_TESTABLE_DECL void SyncIRQHandler (uint16_t myCPUMailBoxId) {
    DefaultTask task = { 0 };
    STATUS_TYPE status =
    SyncMailBoxRead (myCPUMailBoxId, (uint8_t*)&task, sizeof (DefaultTask));

    if (SyncTaskIsValid (&task) == FALSE || status != eSTATUS_SUCCESS) {
        // Invalid task, return early
        return;
    }
    QueueEnqueue (&gSyncTaskQueue, &task);
}

/*
 * \brief Each core needs to call SyncInit
 */
STATUS_TYPE SyncInit (void) {
    // Initialize the task queue
    return QueueInit (&gSyncTaskQueue, gSyncTaskBuffer, SYNC_TASK_QUEUE_SIZE, sizeof (DefaultTask));
}

STATUS_TYPE SyncRegisterHandler (eSYNC_TASKID_TYPE taskID, task_handler_fn_t fn) {
    if ((int32_t)taskID > (int32_t)NUM_TASK_TYPES || fn == NULL) {
        return eSTATUS_FAILURE;
    }
    gHandlers[taskID] = fn;
    return eSTATUS_SUCCESS;
}

STATUS_TYPE SyncProcessTasks (void) {
    if (QueueIsEmpty (&gSyncTaskQueue)) {
        // No tasks to process
        return eSTATUS_SUCCESS;
    }

    DefaultTask task = { 0 };
    while (QueueDequeue (&gSyncTaskQueue, &task) == eSTATUS_SUCCESS) {
        if (SyncTaskIsValid (&task) == FALSE) {
            // Invalid task, skip processing
            continue;
        }
        SyncTaskHeader const* pHeader = (SyncTaskHeader const*)&task;
        task_handler_fn_t fn = SyncGetTaskHandler (pHeader->taskID);
        if (fn != NULL) {
            fn (&task);
        }
    }
    return eSTATUS_SUCCESS;
}

STATUS_TYPE SyncNotifyTaskUartOut (uint16_t len) {
    SyncTaskUartOut task = { 0 };
    task.header.taskID   = eSYNC_TASKID_UART_OUT;
    task.header.magic    = TASK_MAGIC;
    task.len             = len;

    return SyncMailBoxWriteNotify (
    SyncGetOtherCoresMailBoxID (), (uint8_t*)&task, sizeof (SyncTaskUartOut));
}

/*
 * \brief A SEV instruction was executed by CM7
 * and the SEV IRQ handler for CM4 was called.
 * This function was originally defined in CM4/Core/Src/stm32h7xx_it.c
 * by the code generator but I moved it here.
 */
void CM7_SEV_IRQHandler (void) {
    // CPUID == CM4
    SyncIRQHandler (MAILBOX_CM4_ID);
}

/*
 * \brief A sev instruction was executed by CM4
 * and the SEV IRQ handler for CM7 was called.
 * This function was originally defined in CM7/Core/Src/stm32h7xx_it.c
 * by the code generator but I moved it here.
 */
void CM4_SEV_IRQHandler (void) {
    // CPUID == CM7
    SyncIRQHandler (MAILBOX_CM7_ID);
}