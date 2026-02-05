#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "common.h"

#include "drivers/core/lock.h"
#include "drivers/core/log.h"
#include "drivers/core/sys.h"

#include "drivers/serial/serial.h"
#include "drivers/serial/serial_defs.h"

#include "drivers/io/gpio.h"

#include "platform/platform.h"

#include "targets/target.h"

extern uint32_t __SHARED_MEM_BSS_START__;
extern uint32_t __SHARED_MEM_BSS_END__;
extern uint32_t __SHARED_MEM_DATA_START__;
extern uint32_t __SHARED_MEM_DATA_END__;
extern uint32_t __SHARED_MEM_DATA_FLASH_START__;

static eSTATUS_t SharedMem_Init (void) {

    memset (&__SHARED_MEM_BSS_START__, 0, &__SHARED_MEM_BSS_END__ - &__SHARED_MEM_BSS_START__);

    uint32_t const dataStart      = (uint32_t)(&__SHARED_MEM_DATA_START__);
    uint32_t const dataEnd        = (uint32_t)(&__SHARED_MEM_DATA_END__);
    uint32_t const dataSize       = dataEnd - dataStart;
    uint32_t const dataFlashStart = (uint32_t)(&__SHARED_MEM_DATA_FLASH_START__);

    memcpy ((void*)dataStart, (void*)dataFlashStart, dataSize);
    return eSTATUS_OK;
}


eSTATUS_t System_InitPrimaryCore (void) {

    eSTATUS_t status = SharedMem_Init ();
    status |= Plat_System_Init (&e_System, TARG_PRIMARY_CORE, true);
    status |= GPIO_SystemInit ();
    Target_Init ();
    status |= Logger_Init ();
    if (!TARG_DUAL_CORE_ENABLED ()) {
#if TARG_SERIAL_DEBUG_ENABLED()
        status |= SerialDebug_Init ();
#endif
    }
    if (FJ_FAIL (status)) {
        return status;
    }
    PrimaryCore_SetStatus (eCORE_STATUS_SYS_INITED);

    while (TARG_DUAL_CORE_ENABLED () && !SecondaryCore_IsStatus (eCORE_STATUS_SYS_INITED)) {
        // wait for secondary core to init system
    }
    System_SetStatus (eSYS_STATUS_SYS_INITED);
    return status;
}

eSTATUS_t System_InitSecondaryCore (void) {

    while (!PrimaryCore_IsStatus (eCORE_STATUS_SYS_INITED)) {
        // wait for primary core to init system
    }
    eSTATUS_t status = Plat_System_Init (&e_System, TARG_SECONDARY_CORE, false);
#if TARG_SERIAL_DEBUG_ENABLED()
    status |= SerialDebug_Init ();
#endif
    if (FJ_FAIL (status)) {
        return status;
    }
    SecondaryCore_SetStatus (eCORE_STATUS_SYS_INITED);
    return status;
}