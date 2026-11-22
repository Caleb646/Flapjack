#ifndef CORE_COMMON_H
#define CORE_COMMON_H

#include "hal.h"
#include "platform.h"
#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#ifdef UNIT_TEST
#define INLINE
#define STATIC
#else
#define INLINE         inline
#define STATIC         static
#endif

#define FJ_UNUSED(x)             (void)(x)
#define STATIC_ASSERT(expr, msg) static_assert ((expr), msg)

#define ARRAY_SIZE(ARRAY)        (sizeof (ARRAY) / sizeof ((ARRAY)[0]))

typedef int8_t eSTATUS_t;
enum {
    eSTATUS_FAILURE        = -126,
    eSTATUS_BUSY           = -124,
    eSTATUS_TIMEOUT        = -123,
    eSTATUS_NULL_ARG       = -122,
    eSTATUS_INVALID_ARG    = -121,
    eSTATUS_BUS_ERROR      = -120,
    eSTATUS_MEM_ERROR      = -119,
    eSTATUS_HW_ERROR       = -118,
    eSTATUS_DEV_ERROR      = -117,
    eSTATUS_NOT_FOUND      = -116,
    eSTATUS_UNSUPPORTED    = -115,
    eSTATUS_ALREADY_INITED = -114,
    eSTATUS_RETRY          = -113,

    eSTATUS_SUB_STATUS_START__,

    eSTATUS_SUCCESS = 0,
};

// clang-format off

#define STATUS_OK(STATUS)   ((STATUS) == eSTATUS_SUCCESS)
#define STATUS_FAIL(STATUS) ((STATUS) != eSTATUS_SUCCESS)
#define STATUS_RETRY(STATUS) ((STATUS) == eSTATUS_RETRY)

// clang-format on

eSTATUS_t CoreShared_Init (void);

/* Faults last: everything from eCRIT_HARDFAULT on is reported as a fault, by PC rather than file:line. */
typedef enum {
    eCRIT_INIT = 1,
    eCRIT_ASSERT,
    eCRIT_STACK_OVERFLOW,
    eCRIT_HARDFAULT,
    eCRIT_MEMMANAGE,
    eCRIT_BUSFAULT,
    eCRIT_USAGEFAULT,
} eCRIT_t;

/*
 * Records why and where into RAM that survives a reset, then resets the MCU
 * (halting first at a breakpoint if a debugger is attached). The next boot
 * logs the record; it is kept until the next power-on.
 */
#define CRITICAL_ERROR(REASON) CriticalErrorHandler ((REASON), __FILE__, __LINE__)
__attribute__ ((noreturn)) void CriticalErrorHandler (eCRIT_t reason, char const* pFile, uint32_t line);
/*
 * Must be the first thing main() does on each core. A power-on reset leaves the
 * crash record as garbage RAM, and this clears it before anything can read it -
 * and before a crash early in boot could write a record this would then discard.
 */
void CriticalErrorInit (void);
/*
 * Logs the record of the last crash. onlyIfUnreported is the boot call's: it
 * logs a crash once, whereas the shell's call logs whatever is held.
 */
void CriticalErrorReport (bool onlyIfUnreported);
/*
 * Called by CriticalErrorHandler before it resets, so buffered diagnostics are
 * not swallowed by the very fault they explain. Weakly defined as a no-op here;
 * drivers/serial/serial_link.c overrides it to drain the UART synchronously.
 * The weak symbol is what keeps core/ free of any dependency on drivers/.
 */
void CriticalErrorFlushHook (void);
uint32_t GetMilliseconds (void);
uint32_t GetMicroseconds (void);
/*
 * Delay yields to the scheduler once it is running, so the wait can stretch
 * under load and lower-priority work gets the CPU meanwhile. Use it for part
 * settling times, which are minimums.
 *
 * DelayBusyWait always spins. Use it only where the scheduler must not stretch
 * the wait - a protocol whose frame cadence the far end is timing, say - and
 * keep it short: it holds the core at the caller's priority throughout.
 */
void Delay (uint32_t ms);
void DelayBusyWait (uint32_t ms);
void DelayMicroseconds (uint32_t us);

#endif // CORE_COMMON_H
