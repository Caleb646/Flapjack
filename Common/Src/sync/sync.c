#include <string.h>
#include "stm32h7xx_hal.h"

#include "sync/sync.h"
#include "sync/mailbox.h"
#include "mem/mem.h"

#define NUM_TASK_TYPES 2

task_handler_fn_t handlers[NUM_TASK_TYPES];

task_handler_fn_t SyncGetTaskHandler(uint32_t taskID)
{
    if(taskID > NUM_TASK_TYPES) return NULL;
    return handlers[taskID];
}

/*
* \brief Each core needs to call SyncInit
*/
int8_t SyncInit(void)
{
    memset(handlers, 0, sizeof(handlers));
    return 1;
}

int8_t SyncRegisterHandler(task_handler_fn_t fn, uint32_t taskID)
{
    if(taskID > NUM_TASK_TYPES) return -1;
    handlers[taskID] = fn;
    return 1;
}

/*
* \brief A SEV instruction was executed by CM7 
* and the SEV IRQ handler for CM4 was called.
* This function was originally defined in CM4/Core/Src/stm32h7xx_it.c 
* by the code generator but I moved it here.
*/
void CM7_SEV_IRQHandler(void)
{
    // CPUID == CM4
    uint32_t taskID = 0;
    SyncMailBoxRead(MAILBOX_CM4_ID, (uint8_t*)&taskID, sizeof(uint32_t));
    task_handler_fn_t fn = SyncGetTaskHandler(taskID);
    if(fn != NULL)
    {
        fn();
    }
}

/*
* \brief A sev instruction was executed by CM4 
* and the SEV IRQ handler for CM7 was called.
* This function was originally defined in CM7/Core/Src/stm32h7xx_it.c 
* by the code generator but I moved it here.
*/
void CM4_SEV_IRQHandler(void)
{
    // CPUID == CM7
    uint32_t taskID = 0;
    SyncMailBoxRead(MAILBOX_CM7_ID, (uint8_t*)&taskID, sizeof(uint32_t));
    task_handler_fn_t fn = SyncGetTaskHandler(taskID);
    if(fn != NULL)
    {
        fn();
    }
}