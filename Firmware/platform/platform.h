#ifndef __PLATFORM_H
#define __PLATFORM_H

#include <stdbool.h>
#include <stdint.h>

/* Section names are defined by the platform linker script, and Platform_Init
 * zeroes / loads them before anything else runs. */
#ifdef UNIT_TEST
#define SHARED_MEM_BSS_SECTION
#define SHARED_MEM_DATA_SECTION
#define FJ_DECLARE_SHARED(TYPE, NAME)  extern TYPE NAME
#define FJ_DEFINE_SHARED(TYPE, NAME)   TYPE NAME
#else
#define SHARED_MEM_BSS_SECTION         __attribute__ ((section (".shared_mem_bss")))
#define SHARED_MEM_DATA_SECTION        __attribute__ ((section (".shared_mem_data")))
#define FJ_DECLARE_SHARED(TYPE, NAME)  extern TYPE NAME SHARED_MEM_DATA_SECTION
#define FJ_DEFINE_SHARED(TYPE, NAME)   TYPE NAME SHARED_MEM_DATA_SECTION
#endif

/* Neither loaded nor zeroed by startup, so it survives a reset (not a power cycle). */
#ifdef UNIT_TEST
#define NOINIT_SECTION
#else
#define NOINIT_SECTION                 __attribute__ ((section (".noinit"), aligned (8)))
#endif

typedef enum {
    ePLATFORM_FAULT_HARD,
    ePLATFORM_FAULT_MEMMANAGE,
    ePLATFORM_FAULT_BUS,
    ePLATFORM_FAULT_USAGE,
} ePlatformFault_t;

typedef struct {
    ePlatformFault_t fault;
    uint32_t exception; /* what the faulting code was running in; 0 = thread mode */
    uint32_t pc, lr;    /* 0 when the stacked frame could not be read */
    uint32_t cfsr, hfsr, mmfar, bfar;
} PlatformFault_t;

/*
 * Called from the fault vectors with the faulting context captured. Not defined
 * here: the application provides it (core/core_shared.c), which keeps platform
 * free of any dependency on the layers above it.
 */
__attribute__ ((noreturn)) void Platform_OnFault(PlatformFault_t const* pFault);

int Platform_Init(void);

/* Exception number the caller is running in; 0 = thread mode. */
uint32_t Platform_CurrentException(void);

/* Halts at a breakpoint if a debugger is attached, then resets the MCU. */
__attribute__ ((noreturn)) void Platform_Reset(void);

/* True if this boot followed a power-on or brown-out reset, when RAM holds no
 * meaningful contents. Clears this core's reset flags, so it answers once per
 * boot and must be asked before anything else does. */
bool Platform_TakePowerOnReset(void);

/* Free-running 32-bit microsecond counter, shared by both cores. Wraps after
 * ~71.6 min, so take differences, never compare absolute values. */
uint32_t Platform_GetMicroseconds(void);

/* Raises the other core's SEV interrupt. Write the mailbox before calling. */
void Platform_NotifyOtherCore(void);

/* Spin lock shared by both cores. Not recursive. */
bool Platform_LockTake(void);
void Platform_LockRelease(void);

#endif /* __PLATFORM_H */
