#include "sync.h"
#include "common.h"
#include "hal.h"
#include "mem/mem.h"
#include "mem/queue.h"
#include <string.h>

#define TASK_MAGIC          0xBEEFU
#define TASK_QUEUE_CAPACITY 64U

static task_handler_fn_t ga_Handlers[NUMBER_OF_SYNC_TASKS] = { 0 };
QUEUE_DEFINE_STATIC (SyncTask, DefaultTask, TASK_QUEUE_CAPACITY, TRUE);

#ifndef UNIT_TEST

static BOOL_t IsSyncTask_TypeValid (eSYNC_TASKID_t taskID);
static uint8_t IsSyncTask_Valid (DefaultTask const* pTask);
static eSTATUS_t SyncMailBoxWrite (uint32_t mbID, uint8_t const* pBuffer, uint32_t len);
static eSTATUS_t
SyncMailBoxWriteNotify (uint32_t mbID, uint8_t const* pBuffer, uint32_t len);
static eSTATUS_t SyncMailBoxRead (uint32_t mbID, uint8_t* pBuffer, uint32_t len);
static uint16_t SyncGetOtherCoresMailBoxID (void);
static task_handler_fn_t SyncGetTaskHandler (uint32_t taskID);
static void SyncIRQHandler (uint16_t myCPUMailBoxId);

#endif

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

STATIC_TESTABLE_DECL BOOL_t IsSyncTask_TypeValid (eSYNC_TASKID_t taskID) {
    switch (taskID) {
    case eSYNC_TASKID_UART_OUT: break;
    default: return FALSE;
    }
    return TRUE;
}

STATIC_TESTABLE_DECL BOOL_t IsSyncTask_Valid (DefaultTask const* pTask) {
    if (pTask == NULL) {
        return FALSE;
    }
    SyncTaskHeader const* pHeader = (SyncTaskHeader const*)pTask;
    BOOL_t isValid                = TRUE;
    isValid &= (BOOL_t)(pHeader->magic == TASK_MAGIC);
    isValid &= IsSyncTask_TypeValid (pHeader->taskID);
    return isValid;
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

STATIC_TESTABLE_DECL eSTATUS_t SyncMailBoxWrite (uint32_t mbID, uint8_t const* pBuffer, uint32_t len) {
    if (len > MEM_SHARED_MAILBOX_LEN) {
        return eSTATUS_FAILURE;
    }

    uint8_t volatile* pMB = SyncMailBoxGet (mbID);
    memcpy ((void*)pMB, (void*)pBuffer, len);
    return eSTATUS_SUCCESS;
}

STATIC_TESTABLE_DECL eSTATUS_t
SyncMailBoxWriteNotify (uint32_t mbID, uint8_t const* pBuffer, uint32_t len) {
    eSTATUS_t status = SyncMailBoxWrite (mbID, pBuffer, len);
    if (status != eSTATUS_SUCCESS) {
        return eSTATUS_FAILURE;
    }
    asm volatile ("dsb");
    asm volatile ("sev");
    return eSTATUS_SUCCESS;
}

STATIC_TESTABLE_DECL eSTATUS_t SyncMailBoxRead (uint32_t mbID, uint8_t* pBuffer, uint32_t len) {
    if (len > MEM_SHARED_MAILBOX_LEN) {
        return eSTATUS_FAILURE;
    }
    uint8_t volatile* pMB = SyncMailBoxGet (mbID);
    memcpy ((void*)pBuffer, (void*)pMB, len);
    return eSTATUS_SUCCESS;
}


STATIC_TESTABLE_DECL task_handler_fn_t SyncGetTaskHandler (uint32_t taskID) {
    if (IsSyncTask_TypeValid (taskID) != TRUE) {
        return NULL;
    }
    return ga_Handlers[taskID];
}

STATIC_TESTABLE_DECL void SyncIRQHandler (uint16_t myCPUMailBoxId) {
    DefaultTask task = { 0 };
    eSTATUS_t status =
    SyncMailBoxRead (myCPUMailBoxId, (uint8_t*)&task, sizeof (DefaultTask));

    if (IsSyncTask_Valid (&task) == FALSE || status != eSTATUS_SUCCESS) {
        // Invalid task, return early
        return;
    }

    if (SyncTaskQueue_IsFull () == TRUE) {
        return;
    }
    SyncTaskQueue_Push (&task);
}

/*
 * \brief Each core needs to call SyncInit
 */
eSTATUS_t SyncInit (void) {

    if (HAL_GetCurrentCPUID () == CM7_CPUID) {
        // I am running on CM7 so setup the interrupt for CM4 to send me a SEV
        HAL_NVIC_SetPriority (CM4_SEV_IRQn, 10, 10);
        HAL_NVIC_EnableIRQ (CM4_SEV_IRQn);
    } else {
        // I am running on CM4 so setup the interrupt for CM7 to send me a SEV
        HAL_NVIC_SetPriority (CM7_SEV_IRQn, 10, 10);
        HAL_NVIC_EnableIRQ (CM7_SEV_IRQn);
    }
    // Initialize the task queue
    return SyncTaskQueue_Init ();
}

eSTATUS_t SyncRegisterHandler (eSYNC_TASKID_t taskID, task_handler_fn_t fn) {
    if (IsSyncTask_TypeValid (taskID) != TRUE || fn == NULL) {
        return eSTATUS_FAILURE;
    }
    ga_Handlers[taskID] = fn;
    return eSTATUS_SUCCESS;
}

eSTATUS_t SyncProcessTasks (void) {
    if (SyncTaskQueue_IsEmpty () == TRUE) {
        return eSTATUS_SUCCESS;
    }

    DefaultTask task = { 0 };
    while (SyncTaskQueue_Pop (&task) == eSTATUS_SUCCESS) {
        if (IsSyncTask_Valid (&task) == FALSE) {
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

eSTATUS_t SyncNotifyTaskUartOut (uint16_t len) {
    SyncTaskUartOut task = { 0 };
    task.header.taskID   = eSYNC_TASKID_UART_OUT;
    task.header.magic    = TASK_MAGIC;
    task.len             = len;

    return SyncMailBoxWriteNotify (
    SyncGetOtherCoresMailBoxID (), (uint8_t*)&task, sizeof (SyncTaskUartOut));
}