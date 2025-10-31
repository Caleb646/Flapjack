#include "sync.h"
#include "common.h"
#include "hal.h"
#include "mem/mem.h"
#include "mem/queue.h"
#include <stdbool.h>
#include <string.h>

#define TASK_MAGIC                 0xBEEFU
#define TASK_QUEUE_CAPACITY        64U

#define TASK_TYPE_IS_VALID(taskID) ((taskID) < eSYNC_TASKID_MAX)
#define TASK_IS_VALID(pTASK)                                              \
    (                                                                     \
    (pTASK) != NULL && ((SyncTaskHeader*)(pTASK))->magic == TASK_MAGIC && \
    TASK_TYPE_IS_VALID (((SyncTaskHeader*)(pTASK))->taskID)               \
    )

static SHARED_MEM_SECTION MailBox_t ga_MailBoxes[MAILBOX_COUNT] = { 0 };
static task_handler_fn_t ga_Handlers[eSYNC_TASKID_MAX]          = { 0 };
QUEUE_DEFINE_STATIC (SyncTask, DefaultTask, TASK_QUEUE_CAPACITY, true);

// clang-format off

#ifndef UNIT_TEST
static eSTATUS_t SyncMailBoxWrite (uint32_t mbID, uint8_t const* pBuffer, uint32_t len);
static eSTATUS_t SyncMailBoxWriteNotify (uint32_t mbID, uint8_t const* pBuffer, uint32_t len);
static eSTATUS_t SyncMailBoxRead (uint32_t mbID, uint8_t* pBuffer, uint32_t len);
static uint16_t SyncGetOtherCoresMailBoxID (void);
static task_handler_fn_t SyncGetTaskHandler (eSYNC_TASKID_t taskID);
static void SyncIRQHandler (uint16_t myCPUMailBoxId);
#endif

// clang-format on

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

STATIC_TESTABLE_DECL uint16_t SyncGetOtherCoresMailBoxID (void) {

    if (HAL_GetCurrentCPUID () == CM7_CPUID) {
        return MAILBOX_CM4_ID;
    }
    return MAILBOX_CM7_ID;
}

STATIC_TESTABLE_DECL vMailBox_t* SyncMailBoxGet (uint32_t mbID) {

    if (mbID >= MAILBOX_COUNT) {
        return NULL;
    }
    return &ga_MailBoxes[mbID];
}

STATIC_TESTABLE_DECL eSTATUS_t SyncMailBoxWrite (uint32_t mbID, uint8_t const* pBuffer, uint32_t len) {

    if (len > sizeof (MailBox_t) || pBuffer == NULL) {
        return eSTATUS_FAILURE;
    }
    memcpy ((void*)SyncMailBoxGet (mbID), (void*)pBuffer, len);
    return eSTATUS_SUCCESS;
}

STATIC_TESTABLE_DECL eSTATUS_t SyncMailBoxWriteNotify (uint32_t mbID, uint8_t const* pBuffer, uint32_t len) {

    eSTATUS_t status = SyncMailBoxWrite (mbID, pBuffer, len);
    if (status != eSTATUS_SUCCESS) {
        return eSTATUS_FAILURE;
    }

#ifndef UNIT_TEST

    asm volatile ("dsb");
    asm volatile ("sev");

#endif
    return eSTATUS_SUCCESS;
}

STATIC_TESTABLE_DECL eSTATUS_t SyncMailBoxRead (uint32_t mbID, uint8_t* pBuffer, uint32_t len) {

    if (len > sizeof (MailBox_t) || pBuffer == NULL) {
        return eSTATUS_FAILURE;
    }
    memcpy ((void*)pBuffer, (void*)SyncMailBoxGet (mbID), len);
    return eSTATUS_SUCCESS;
}


STATIC_TESTABLE_DECL task_handler_fn_t SyncGetTaskHandler (eSYNC_TASKID_t taskId) {

    if (TASK_TYPE_IS_VALID (taskId) == false) {
        return NULL;
    }
    return ga_Handlers[taskId];
}

STATIC_TESTABLE_DECL void SyncIRQHandler (uint16_t myCPUMailBoxId) {

    DefaultTask task = { 0 };
    eSTATUS_t status = SyncMailBoxRead (myCPUMailBoxId, (uint8_t*)&task, sizeof (DefaultTask));

    if (TASK_IS_VALID (&task) == false || status != eSTATUS_SUCCESS) {
        return;
    }

    if (SyncTaskQueue_IsFull () == true) {
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

    if (TASK_TYPE_IS_VALID (taskID) == false || fn == NULL) {
        return eSTATUS_FAILURE;
    }
    ga_Handlers[taskID] = fn;
    return eSTATUS_SUCCESS;
}

eSTATUS_t SyncProcessTasks (void) {

    if (SyncTaskQueue_IsEmpty () == true) {
        return eSTATUS_SUCCESS;
    }

    DefaultTask task = { 0 };
    while (SyncTaskQueue_Pop (&task) == eSTATUS_SUCCESS) {
        if (TASK_IS_VALID (&task) == false) {
            continue;
        }
        SyncTaskHeader const* pHeader = (SyncTaskHeader const*)&task;
        task_handler_fn_t fn          = SyncGetTaskHandler (pHeader->taskID);
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

    return SyncMailBoxWriteNotify (SyncGetOtherCoresMailBoxID (), (uint8_t*)&task, sizeof (SyncTaskUartOut));
}


#define DEFAULT_SEMAPHORE_ID 1U

bool LockTake (void) {

    while (HAL_HSEM_FastTake (DEFAULT_SEMAPHORE_ID) != HAL_OK) {
    }
    return true;
}

void LockRelease (void) {

    HAL_HSEM_Release (DEFAULT_SEMAPHORE_ID, 0);
}
