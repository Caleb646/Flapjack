#ifndef DRIVERS_SYS_H
#define DRIVERS_SYS_H

#include <stdbool.h>
#include <stdint.h>

#include "common.h"

#include "drivers/core/lock.h"

#include "platform/platform.h"

#include "targets/target.h"

typedef uint16_t volatile sys_status_t;
typedef uint16_t volatile eSYS_STATUS_t;
enum {
    eSYS_STATUS_RESET          = 0U,
    eSYS_STATUS_SYS_INITED     = (1U << 0U),
    eSYS_STATUS_HWDEVS_INITED  = (1U << 1U),
    eSYS_STATUS_EXTDEVS_INITED = (1U << 2U),
    eSYS_STATUS_RUNNING        = (1U << 3U),


};

typedef uint16_t volatile core_status_t;
typedef uint16_t volatile eCORE_STATUS_t;
enum {
    eCORE_STATUS_RESET      = 0U,
    eCORE_STATUS_SYS_INITED = (1U << 0U),


};

typedef struct Core_s {
    uint32_t id;
    uint32_t pendIrqId;
    uint32_t sevIrqId;
    core_status_t statusFlags;
} Core_t;

typedef struct System_s {
    SpinLock_t sysLock;
    sys_status_t statusFlags;
} System_t;

FJ_DECLARE_SHARED (Core_t, e_Cores[]);
FJ_DECLARE_SHARED (uint8_t, e_nCores);
FJ_DECLARE_SHARED (System_t, e_System);

eSTATUS_t Plat_System_Init (System_t* pSystem, uint32_t coreId, bool isPrimary);

eSTATUS_t System_InitPrimaryCore (void);
eSTATUS_t System_InitSecondaryCore (void);

static inline void System_SetStatus (eSYS_STATUS_t status) {

    SpinLock_Take (&e_System.sysLock);
    e_System.statusFlags |= status;
    SpinLock_Release (&e_System.sysLock);
}

static inline bool System_IsStatus (eSYS_STATUS_t status) {
    return (e_System.statusFlags & status) == status;
}

#define PRIMARY_CORE_INDEX   0U
#define SECONDARY_CORE_INDEX 1U
// clang-format off
#define CORE_ID_TO_INDEX(CORE_ID) ((CORE_ID) == TARG_PRIMARY_CORE ? PRIMARY_CORE_INDEX : SECONDARY_CORE_INDEX)
// clang-format on

uint32_t Plat_CurrentCore_GetId (void);

static inline uint32_t CurrentCore_GetId (void) {
    return Plat_CurrentCore_GetId ();
}

static inline Core_t* Core_GetMutable (uint32_t coreId) {
    return &e_Cores[CORE_ID_TO_INDEX (coreId)];
}

static inline Core_t* CurrentCore_GetMutable (void) {
    return Core_GetMutable (CurrentCore_GetId ());
}

static inline void Core_SetStatus (uint32_t coreId, eCORE_STATUS_t status) {
    Core_GetMutable (coreId)->statusFlags |= status;
}

static inline bool Core_IsStatus (uint32_t coreId, eCORE_STATUS_t status) {
    return (Core_GetMutable (coreId)->statusFlags & status) == status;
}

static inline bool PrimaryCore_IsStatus (eCORE_STATUS_t status) {
    return Core_IsStatus (TARG_PRIMARY_CORE, status);
}

static inline void PrimaryCore_SetStatus (eCORE_STATUS_t status) {
    Core_SetStatus (TARG_PRIMARY_CORE, status);
}

static inline bool SecondaryCore_IsStatus (eCORE_STATUS_t status) {
    return Core_IsStatus (TARG_SECONDARY_CORE, status);
}

static inline void SecondaryCore_SetStatus (eCORE_STATUS_t status) {
    Core_SetStatus (TARG_SECONDARY_CORE, status);
}

#define CORE_IS_PRIMARY()   (CurrentCore_GetId () == TARG_PRIMARY_CORE)
#define CORE_IS_SECONDARY() (CurrentCore_GetId () == TARG_SECONDARY_CORE)


#endif // DRIVERS_SYS_H