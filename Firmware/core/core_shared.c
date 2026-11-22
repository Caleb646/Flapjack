#include "core/core.h"
#include "hal.h"

#ifndef UNIT_TEST
#include "FreeRTOS.h"
#include "task.h"
#endif // UNIT_TEST
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

typedef struct {
    uint32_t magic;
    uint32_t reason;
    uint32_t line;
    uint32_t uptimeMs;
    uint32_t exception; /* IPSR of the failing context; 0 = a task (thread mode) */
    uint32_t reported;
    char file[24];
    char task[16];
    uint32_t pc, lr, cfsr, hfsr, mmfar, bfar; /* faults only */
} CrashRecord_t;

/*
 * Whole 64-bit words because H7 SRAM ECC is computed per 64 bits: the
 * power-on clear has to leave every word of the record with valid ECC.
 */
STATIC_ASSERT (sizeof (CrashRecord_t) % 8U == 0U, "crash record must be whole 64-bit words");

/* The size is folded in so a record left by a build with another layout fails the check. */
#define CRASH_MAGIC (0xC2A5E000U | (uint32_t)sizeof (CrashRecord_t))

static CrashRecord_t s_crash NOINIT_SECTION;
static bool volatile s_inCriticalError = false;

static char const* const s_reasonNames[] = {
    [eCRIT_INIT]           = "INIT",
    [eCRIT_ASSERT]         = "ASSERT",
    [eCRIT_STACK_OVERFLOW] = "STACK_OVERFLOW",
    [eCRIT_HARDFAULT]      = "HARDFAULT",
    [eCRIT_MEMMANAGE]      = "MEMMANAGE",
    [eCRIT_BUSFAULT]       = "BUSFAULT",
    [eCRIT_USAGEFAULT]     = "USAGEFAULT",
};

static bool SanityCheckTimers (void);

eSTATUS_t CoreShared_Init (void) {

    if (SanityCheckTimers () == false) {
        return eSTATUS_FAILURE;
    }

    return eSTATUS_SUCCESS;
}

__attribute__ ((weak)) void CriticalErrorFlushHook (void) {
}

static void CopyString (char* pDst, uint32_t size, char const* pSrc) {

    uint32_t i = 0U;
    for (; (i + 1U) < size && pSrc[i] != '\0'; ++i) {
        pDst[i] = pSrc[i];
    }
    pDst[i] = '\0';
}

/* pFault is the captured fault, NULL when not called from one. */
static void CriticalErrorRecord (eCRIT_t reason, char const* pFile, uint32_t line, PlatformFault_t const* pFault) {

    CrashRecord_t* const p = &s_crash;

    p->magic     = 0U;
    p->reason    = (uint32_t)reason;
    p->line      = line;
    p->uptimeMs  = GetMilliseconds ();
    p->reported  = 0U;
    p->exception = Platform_CurrentException ();
    p->pc = p->lr = p->cfsr = p->hfsr = p->mmfar = p->bfar = 0U;

    if (pFault != NULL) {
        p->exception = pFault->exception;
        p->pc        = pFault->pc;
        p->lr        = pFault->lr;
        p->cfsr      = pFault->cfsr;
        p->hfsr      = pFault->hfsr;
        p->mmfar     = pFault->mmfar;
        p->bfar      = pFault->bfar;
    }

    /* __FILE__ is the full build path; the basename is what fits and what reads. */
    char const* pBase = (pFile != NULL) ? pFile : "";
    for (char const* pc = pBase; *pc != '\0'; ++pc) {
        if (*pc == '/' || *pc == '\\') {
            pBase = pc + 1;
        }
    }
    CopyString (p->file, sizeof (p->file), pBase);
    CopyString (p->task, sizeof (p->task), "-");

#ifndef UNIT_TEST
    /* Also right for a stack overflow: FreeRTOS checks the outgoing task from
     * PendSV, before it switches, so the current task is the one that overflowed. */
    if (xTaskGetSchedulerState () != taskSCHEDULER_NOT_STARTED) {
        CopyString (p->task, sizeof (p->task), pcTaskGetName (NULL));
    }
#endif // UNIT_TEST

    p->magic = CRASH_MAGIC;
}

__attribute__ ((noreturn)) static void CriticalErrorFinish (eCRIT_t reason, char const* pFile, uint32_t line, PlatformFault_t const* pFault) {

    /* A second failure while handling the first - the flush faulting, say -
     * must not overwrite the record of the one that started it. */
    if (!s_inCriticalError) {
        s_inCriticalError = true;
        CriticalErrorRecord (reason, pFile, line, pFault);
        /* Not from a fault: a fault is when the log ring is least likely to be
         * intact, and a HardFault handler that faults locks up rather than resetting. */
        if (pFault == NULL) {
            CriticalErrorFlushHook ();
        }
    }

    __disable_irq ();
    Platform_Reset ();
}

void CriticalErrorHandler (eCRIT_t reason, char const* pFile, uint32_t line) {

    CriticalErrorFinish (reason, pFile, line, NULL);
}

void Platform_OnFault (PlatformFault_t const* pFault) {

    eCRIT_t reason = eCRIT_HARDFAULT;
    switch (pFault->fault) {
    case ePLATFORM_FAULT_MEMMANAGE: reason = eCRIT_MEMMANAGE; break;
    case ePLATFORM_FAULT_BUS: reason = eCRIT_BUSFAULT; break;
    case ePLATFORM_FAULT_USAGE: reason = eCRIT_USAGEFAULT; break;
    case ePLATFORM_FAULT_HARD: break;
    }
    CriticalErrorFinish (reason, NULL, 0U, pFault);
}

void CriticalErrorInit (void) {

    if (Platform_TakePowerOnReset ()) {
        memset (&s_crash, 0, sizeof (s_crash));
    }
}

void CriticalErrorReport (bool onlyIfUnreported) {

    CrashRecord_t* const p = &s_crash;

    if (p->magic != CRASH_MAGIC) {
        if (!onlyIfUnreported) {
            LOG_INFO ("No crash recorded since power-on");
        }
        return;
    }
    if (onlyIfUnreported && p->reported != 0U) {
        return;
    }
    p->reported = 1U;

    char const* pName = "?";
    if (p->reason < ARRAY_SIZE (s_reasonNames) && s_reasonNames[p->reason] != NULL) {
        pName = s_reasonNames[p->reason];
    }

    if (p->reason >= (uint32_t)eCRIT_HARDFAULT) {
        LOG_ERROR (
        "Crash: %s pc=0x%x lr=0x%x cfsr=0x%x hfsr=0x%x mmfar=0x%x bfar=0x%x task=%s exc=%u up=%ums",
        pName,
        (unsigned)p->pc,
        (unsigned)p->lr,
        (unsigned)p->cfsr,
        (unsigned)p->hfsr,
        (unsigned)p->mmfar,
        (unsigned)p->bfar,
        p->task,
        (unsigned)p->exception,
        (unsigned)p->uptimeMs
        );
    } else {
        LOG_ERROR (
        "Crash: %s at %s:%u task=%s exc=%u up=%ums",
        pName,
        p->file,
        (unsigned)p->line,
        p->task,
        (unsigned)p->exception,
        (unsigned)p->uptimeMs
        );
    }
}

void __assert_func (const char* file, int line, const char* func, const char* failedexpr) {

    FJ_UNUSED (func);
    FJ_UNUSED (failedexpr);

    CriticalErrorHandler (eCRIT_ASSERT, file, (uint32_t)line);
}

static bool SanityCheckTimers (void) {

    uint32_t const msDelay = 10;
    uint32_t tempStart     = GetMilliseconds ();
    HAL_Delay (msDelay);
    uint32_t tempEnd = GetMilliseconds ();

    if (tempEnd == tempStart || tempEnd < tempStart) {
        return false;
    }

    if ((tempEnd - tempStart) < msDelay - 2U || (tempEnd - tempStart) > msDelay + 2U) {
        return false;
    }

    uint32_t const usDelay = msDelay * 1000U;
    tempStart              = GetMicroseconds ();
    DelayMicroseconds (usDelay);
    tempEnd = GetMicroseconds ();

    if (tempEnd == tempStart || tempEnd < tempStart) {
        return false;
    }

    if ((tempEnd - tempStart) < usDelay - (2U * 1000U) || (tempEnd - tempStart) > usDelay + (2U * 1000U)) {
        return false;
    }

    return true;
}

// Can't be used inside a disable IRQ section because it uses HAL_GetTick.
uint32_t GetMilliseconds (void) {
    return HAL_GetTick ();
}

uint32_t GetMicroseconds (void) {
    return Platform_GetMicroseconds ();
}

void Delay (uint32_t ms) {

#ifndef UNIT_TEST

    /*
     * Block rather than spin once there is a scheduler to yield to. A busy-wait
     * holds the core at the caller's priority for the whole delay and starves
     * everything below it - Imu_Task's init retry spends ~2.5 s per attempt
     * inside IMUSoftReset, all of it here.
     *
     * Before vTaskStartScheduler, inside an ISR, or with the scheduler
     * suspended there is nothing to yield to, so spin as before.
     *
     * The +1 tick is not slop. vTaskDelay counts whole ticks from the next tick
     * boundary, so pdMS_TO_TICKS alone can return up to one tick early; every
     * caller uses this to satisfy a part's settling time, which is a minimum.
     * At configTICK_RATE_HZ 1000 that costs 1 ms per call.
     */
    if (xTaskGetSchedulerState () == taskSCHEDULER_RUNNING && xPortIsInsideInterrupt () == pdFALSE) {
        vTaskDelay (pdMS_TO_TICKS (ms) + 1U);
        return;
    }

#endif // UNIT_TEST

    DelayBusyWait (ms);
}

void DelayBusyWait (uint32_t ms) {

    DelayMicroseconds (ms * 1000U);
}

void DelayMicroseconds (uint32_t us) {

#ifndef UNIT_TEST

    uint32_t usStart = GetMicroseconds ();
    while ((GetMicroseconds () - usStart) < us) {
        __NOP ();
    }

#endif // UNIT_TEST
}
