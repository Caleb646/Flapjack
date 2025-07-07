#include "sync.h"
#include "common.h"
#include "mem/mem.h"
#include <string.h>

#define NUM_TASK_TYPES       (2U)
#define NUM_TASK_PER_TYPE    (4U)
#define SYNC_TASK_QUEUE_SIZE (16U)
#define TASK_MAGIC           (0xBEEF)

typedef struct {
    uint64_t padding;
} DefaultTask;

STATIC_ASSERT (sizeof (DefaultTask) == 8U, "");

typedef struct {
    DefaultTask tasks[SYNC_TASK_QUEUE_SIZE];
    uint8_t head;
    uint8_t tail;
    uint8_t count;
} SyncTaskQueue;

task_handler_fn_t gHandlers[NUM_TASK_TYPES] = { 0 };
SyncTaskQueue gSyncTaskQueue                = { 0 };

STATIC_TESTABLE_DECL void SyncTaskQueue_Init (SyncTaskQueue* q) {
    q->head  = 0;
    q->tail  = 0;
    q->count = 0;
}

STATIC_TESTABLE_DECL uint8_t SyncTaskQueue_IsEmpty (SyncTaskQueue const* q) {
    return q->count == 0;
}

STATIC_TESTABLE_DECL uint8_t SyncTaskQueue_IsFull (SyncTaskQueue const* q) {
    return q->count == SYNC_TASK_QUEUE_SIZE;
}

STATIC_TESTABLE_DECL STATUS_TYPE SyncTaskQueue_Enqueue (SyncTaskQueue* q, DefaultTask const* task) {
    if (SyncTaskQueue_IsFull (q)) {
        return eSTATUS_FAILURE;
    }
    q->tasks[q->tail] = *task;
    q->tail = ((uint8_t)(q->tail + 1)) & (uint8_t)(SYNC_TASK_QUEUE_SIZE - 1);
    q->count++;
    return eSTATUS_SUCCESS;
}

STATIC_TESTABLE_DECL STATUS_TYPE SyncTaskQueue_Dequeue (SyncTaskQueue* q, DefaultTask* out_task) {
    if (SyncTaskQueue_IsEmpty (q)) {
        return eSTATUS_FAILURE;
    }
    *out_task = q->tasks[q->head];
    q->head = ((uint8_t)(q->head + 1)) & (uint8_t)(SYNC_TASK_QUEUE_SIZE - 1);
    q->count--;
    return eSTATUS_SUCCESS;
}

STATIC_TESTABLE_DECL STATUS_TYPE SyncTaskQueue_Peek (SyncTaskQueue const* q, DefaultTask* out_task) {
    if (SyncTaskQueue_IsEmpty (q)) {
        return eSTATUS_FAILURE;
    }
    *out_task = q->tasks[q->head];
    return eSTATUS_SUCCESS;
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

/*
 * \brief Each core needs to call SyncInit
 */
STATUS_TYPE SyncInit (void) {
    // memset (gHandlers, 0, sizeof (gHandlers));
    return eSTATUS_SUCCESS;
}

STATUS_TYPE SyncRegisterHandler (eSYNC_TASKID_TYPE taskID, task_handler_fn_t fn) {
    if (taskID > NUM_TASK_TYPES || fn == NULL) {
        return eSTATUS_FAILURE;
    }
    gHandlers[taskID] = fn;
    return eSTATUS_SUCCESS;
}

/*
 * \brief A SEV instruction was executed by CM7
 * and the SEV IRQ handler for CM4 was called.
 * This function was originally defined in CM4/Core/Src/stm32h7xx_it.c
 * by the code generator but I moved it here.
 */
void CM7_SEV_IRQHandler (void) {
    // CPUID == CM4
    uint32_t taskID = 0;
    SyncMailBoxRead (MAILBOX_CM4_ID, (uint8_t*)&taskID, sizeof (uint32_t));
    task_handler_fn_t fn = SyncGetTaskHandler (taskID);
    if (fn != NULL) {
        fn ();
    }
}

/*
 * \brief A sev instruction was executed by CM4
 * and the SEV IRQ handler for CM7 was called.
 * This function was originally defined in CM7/Core/Src/stm32h7xx_it.c
 * by the code generator but I moved it here.
 */
void CM4_SEV_IRQHandler (void) {
    // CPUID == CM7
    uint32_t taskID = 0;
    SyncMailBoxRead (MAILBOX_CM7_ID, (uint8_t*)&taskID, sizeof (uint32_t));
    task_handler_fn_t fn = SyncGetTaskHandler (taskID);
    if (fn != NULL) {
        fn ();
    }
}