#include "core/log/logger.h"
#include "core/core_shared.h"
#include "core/log/format.h"
#include "core/sync.h"
#include "hal.h"
#include "mem/mem.h"
#include "mem/ring_buff.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>


// #ifdef __GNUC__
// #define PUTCHAR_PROTOTYPE int __io_putchar (int ch)
// #else
// #define PUTCHAR_PROTOTYPE int fputc (int ch, FILE* f)
// #endif

#if LOGGER_SHOULD_BLOCK_ON_OVERWRITE == 1
#define LOGGER_WRITE_CHAR(ch) LoggerWriteChar_Blocking (ch)
#else
#define LOGGER_WRITE_CHAR(ch) LoggerWriteChar_NonBlocking (ch)
#endif

#define PRIMARY_LOGGER_IS_ME() (HAL_GetCurrentCPUID () == PRIMARY_LOGGER_ROLE)
#define TEMP_BUFFER_SIZE       512U
#define MAX_NSINKS             4U

// NOLINTBEGIN
static SHARED_MEM_SECTION LoggerWriteToSink_t gLoggerSinks[MAX_NSINKS] = { 0 };
static SHARED_MEM_SECTION uint8_t gCurrentSinkIdx                      = 0U;
static SHARED_MEM_SECTION uint8_t gCM4RingBufStorage[1024]             = { 0 };
static SHARED_MEM_SECTION uint8_t gCM7RingBufStorage[4096]             = { 0 };
static SHARED_MEM_SECTION RingBuff g_CM4_RingBuf                       = { 0 };
static SHARED_MEM_SECTION RingBuff g_CM7_RingBuf                       = { 0 };
static SHARED_MEM_SECTION bool gLoggerInitialized                      = false;
// static uint8_t ga_TempReadBuffer[TEMP_BUFFER_SIZE]                     = { 0 };

// typedef RingBuff volatile vRingBuff_t;
typedef RingBuff vRingBuff_t;
// NOLINTEND

static eSTATUS_t LoggerSyncUARTTaskHandler (DefaultTask const* pTask);
static eSTATUS_t LoggerWriteToSinks (vRingBuff_t* pRingBuf, uint32_t totalLen);
static vRingBuff_t* LoggerGetMyRingBuf (void);
static vRingBuff_t* LoggerGetOtherRingBuf (void);
static void LoggerWriteChar_Blocking (char ch);
static void LoggerWriteChar_NonBlocking (char ch);
static void LoggerWriteChar (void* p, char ch);

static eSTATUS_t LoggerSyncUARTTaskHandler (DefaultTask const* pTask) {
    // Write the other core's ring buffer out to the UART
    if (PRIMARY_LOGGER_IS_ME () == true) {
        SyncTaskUartOut const* pSyncTaskUartOut = (SyncTaskUartOut const*)pTask;
        return LoggerWriteToSinks (LoggerGetOtherRingBuf (), pSyncTaskUartOut->len);
    }
    return eSTATUS_SUCCESS;
}

static eSTATUS_t LoggerWriteToSinks (vRingBuff_t* pRingBuf, uint32_t totalLen) {

    if (RingBuffIsValid (pRingBuf) != true) {
        return eSTATUS_FAILURE;
    }

    uint32_t nTotalBytes   = totalLen;
    uint32_t maxIterations = 100;
    while (nTotalBytes != 0U && maxIterations-- > 0) {
        /*
         * Read totalLen bytes from ringbuffer and this will include bytes
         * that overflowed (wrapped around to the beginning of the buffer)
         */
        // uint32_t toWrite   = MIN_U32 (nTotalBytes, TEMP_BUFFER_SIZE);
        // uint32_t bytesRead = RingBuffRead (pRingBuf, (void*)ga_TempReadBuffer, toWrite);

        uint32_t bytesRead = RingBuffGetLinearBlockReadLength (pRingBuf);
        void* pLinearRead  = RingBuffGetLinearBlockReadAddress (pRingBuf);

        for (uint32_t i = 0; i < gCurrentSinkIdx; ++i) {
            if (gLoggerSinks[i] != NULL) {
                gLoggerSinks[i](pLinearRead, bytesRead);
            }
        }
        RingBuffSkip (pRingBuf, bytesRead);
        nTotalBytes -= bytesRead;
    }

    return eSTATUS_SUCCESS;
}

/*
 * Returns the CURRENT core's ring buffer
 */
static vRingBuff_t* LoggerGetMyRingBuf (void) {
    return (HAL_GetCurrentCPUID () == CM7_CPUID) ? &g_CM7_RingBuf : &g_CM4_RingBuf;
}

/*
 * Returns the OTHER core's ring buffer
 */
static vRingBuff_t* LoggerGetOtherRingBuf (void) {
    return (HAL_GetCurrentCPUID () == CM7_CPUID) ? &g_CM4_RingBuf : &g_CM7_RingBuf;
}

static void LoggerWriteChar_Blocking (char ch) {

    vRingBuff_t* pMyRingBuf = LoggerGetMyRingBuf ();
    /*
     * If the char will cause an overwrite in the ring buffer:
     *   Primary Logger = write what is currently in ring buffer to uart
     *   Secondary Logger = wait until primary logger writes my ring buffer out to the uart and makes more space.
     */
    if (RingBuffGetFree (pMyRingBuf) == 0U) {
        if (PRIMARY_LOGGER_IS_ME () == true) {
            LoggerWriteToSinks (pMyRingBuf, RingBuffGetFull (pMyRingBuf));
        } else {
            int32_t timeout = 10000;
            while (timeout-- > 0) {
                if (RingBuffGetFree (pMyRingBuf) > 0) {
                    break;
                }
            }
        }
    }
    LoggerWriteChar_NonBlocking (ch);
}

static void LoggerWriteChar_NonBlocking (char ch) {

    static int tempCount = 0;
    ++tempCount;

    vRingBuff_t* pMyRingBuf = LoggerGetMyRingBuf ();
    RingBuffWrite (pMyRingBuf, (void*)&ch, 1U);

    if (ch == '\n') {
        if (PRIMARY_LOGGER_IS_ME () == true) {
            LoggerWriteToSinks (pMyRingBuf, RingBuffGetFull (pMyRingBuf));
        } else {
            SyncNotifyTaskUartOut (RingBuffGetFull (pMyRingBuf));
        }
    }
}

/*
 * Used by printf impl in format.c
 */
static void LoggerWriteChar (void* p, char ch) {

    FJ_UNUSED (p);

    // if (PRIMARY_LOGGER_IS_ME () == true) {
    //     return;
    // }

    LOGGER_WRITE_CHAR (ch);
}

eSTATUS_t LoggerAddSink (LoggerWriteToSink_t sink) {

    if (sink == NULL || gCurrentSinkIdx >= MAX_NSINKS) {
        return eSTATUS_FAILURE;
    }
    gLoggerSinks[gCurrentSinkIdx++] = sink;
    return eSTATUS_SUCCESS;
}

eSTATUS_t LoggerRemoveSink (LoggerWriteToSink_t fpSink) {

    if (fpSink == NULL || gCurrentSinkIdx == 0U) {
        return eSTATUS_FAILURE;
    }
    for (uint32_t i = 0; i < gCurrentSinkIdx; ++i) {
        if (gLoggerSinks[i] == fpSink) {

            gLoggerSinks[i] = NULL;
            // Shift remaining sinks down
            for (uint32_t j = i; j < gCurrentSinkIdx - 1; ++j) {
                gLoggerSinks[j] = gLoggerSinks[j + 1];
            }
            --gCurrentSinkIdx;
            return eSTATUS_SUCCESS;
        }
    }
    return eSTATUS_FAILURE;
}

eSTATUS_t LoggerInit (void) {

    init_printf (NULL, LoggerWriteChar);
    // Let both cores register this task handler only the primary logger
    // core will actually write to the UART
    if (SyncRegisterHandler (eSYNC_TASKID_UART_OUT, LoggerSyncUARTTaskHandler) != eSTATUS_SUCCESS) {
        return eSTATUS_FAILURE;
    }

    if (gLoggerInitialized == true) {
        return eSTATUS_SUCCESS;
    }

    ATOMIC_BLOCK_LOCAL (eNVIC_PRIO_LVL_MAX) {

        gLoggerInitialized = true;
        LockTake ();

        gCurrentSinkIdx = 0U;
        memset ((void*)gLoggerSinks, 0, sizeof (gLoggerSinks));

        bool ok = true;
        ok &= RingBuffInit (gCM4RingBufStorage, sizeof (gCM4RingBufStorage), &g_CM4_RingBuf);
        ok &= RingBuffInit (gCM7RingBufStorage, sizeof (gCM7RingBufStorage), &g_CM7_RingBuf);
        if (ok != true) {
            gLoggerInitialized = false;
            return eSTATUS_FAILURE;
        }
        LockRelease ();
    }

    return eSTATUS_SUCCESS;
}