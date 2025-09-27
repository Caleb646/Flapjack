#include "log/logger.h"
#include "common.h"
#include "conf/board.h"
#include "conf/conf.h"
#include "hal.h"
#include "log/format.h"
#include "mem/mem.h"
#include "mem/ring_buff.h"
#include "peripheral/uart.h"
#include "sync.h"
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

#define PRIMARY_LOGGER_IS_ME() \
    (HAL_GetCurrentCPUID () == PRIMARY_LOGGER_ROLE)
#define TEMP_BUFFER_SIZE 512U
#define MAX_NSINKS       4U

// NOLINTBEGIN
static SHARED_MEM_SECTION LoggerWriteToSink_t gLoggerSinks[MAX_NSINKS] = { 0 };
static SHARED_MEM_SECTION uint8_t gCurrentSinkIdx          = 0U;
static SHARED_MEM_SECTION uint8_t gCM4RingBufStorage[1024] = { 0 };
static SHARED_MEM_SECTION uint8_t gCM7RingBufStorage[4096] = { 0 };
static SHARED_MEM_SECTION RingBuff* gp_CM4_RingBuf         = NULL;
static SHARED_MEM_SECTION RingBuff* gp_CM7_RingBuf         = NULL;
static uint8_t ga_TempReadBuffer[TEMP_BUFFER_SIZE]         = { 0 };

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
        return LoggerWriteToSinks (
        LoggerGetOtherRingBuf (),
        pSyncTaskUartOut->len
        );
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
        uint32_t toWrite = MIN_U32 (nTotalBytes, TEMP_BUFFER_SIZE);
        uint32_t bytesRead =
        RingBuffRead (pRingBuf, (void*)ga_TempReadBuffer, toWrite);
        for (uint32_t i = 0; i < gCurrentSinkIdx; ++i) {
            if (gLoggerSinks[i] != NULL) {
                gLoggerSinks[i](ga_TempReadBuffer, bytesRead);
            }
        }
        nTotalBytes -= bytesRead;
    }

    return eSTATUS_SUCCESS;
}

/*
 * Returns the CURRENT core's ring buffer
 */
static vRingBuff_t* LoggerGetMyRingBuf (void) {
    return (HAL_GetCurrentCPUID () == CM7_CPUID) ? gp_CM7_RingBuf : gp_CM4_RingBuf;
}

/*
 * Returns the OTHER core's ring buffer
 */
static vRingBuff_t* LoggerGetOtherRingBuf (void) {
    return (HAL_GetCurrentCPUID () == CM7_CPUID) ? gp_CM4_RingBuf : gp_CM7_RingBuf;
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

    vRingBuff_t* pMyRingBuf = LoggerGetMyRingBuf ();
    RingBuffWrite (pMyRingBuf, (void*)&ch, 1);

    if ((char)ch == '\n') {
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
    LOGGER_WRITE_CHAR ((char)ch);
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

// PUTCHAR_PROTOTYPE {
//     LOGGER_WRITE_CHAR ((char)ch);
//     return ch;
// }

eSTATUS_t LoggerInit (void) {

    init_printf (NULL, LoggerWriteChar);
    // Let both cores register this task handler only the primary logger
    // core will actually write to the UART
    if (SyncRegisterHandler (eSYNC_TASKID_UART_OUT, LoggerSyncUARTTaskHandler) !=
        eSTATUS_SUCCESS) {
        return eSTATUS_FAILURE;
    }

    if (PRIMARY_LOGGER_IS_ME () == true) {
        bool ok = true;
        ok &= RingBuffInit (gp_CM4_RingBuf, gCM4RingBufStorage, sizeof (gCM4RingBufStorage));
        ok &= RingBuffInit (gp_CM7_RingBuf, gCM7RingBufStorage, sizeof (gCM7RingBufStorage));
        if (ok != true) {
            return eSTATUS_FAILURE;
        }
    }
    return eSTATUS_SUCCESS;
}


static SHARED_MEM_SECTION SerialDebug_t g_SerialDebug = { 0 };

static void SerialDebugSink (uint8_t const* pData, uint32_t len) {

    if (UARTIsValid (g_SerialDebug.busId) == true) {
        UARTWrite_Blocking (g_SerialDebug.busId, pData, len);
    }
}

eSTATUS_t SerialDebugInit (SerialDebugInitConf_t conf, DeviceBoardConf_t devBoardConf) {

    if (g_SerialDebug.isInitialized == true) {
        return eSTATUS_FAILURE;
    }

    if (devBoardConf.pBusBoardConf == NULL) {
        return eSTATUS_FAILURE;
    }

    eBUS_ID_t busId       = devBoardConf.pBusBoardConf->busId;
    eDEVICE_ID_t deviceId = conf.deviceId;
    eSTATUS_t status      = eSTATUS_SUCCESS;
    memset (&g_SerialDebug, 0, sizeof (g_SerialDebug));
    g_SerialDebug.busId    = busId;
    g_SerialDebug.deviceId = deviceId;

    if (BUS_ID_IS_UART (busId) == true) {

        UARTBoardConf_t* pUARTBoardConf = (UARTBoardConf_t*)devBoardConf.pBusBoardConf;
        uint32_t baudRate = pUARTBoardConf->baudRate;
        (void)baudRate;
        UART_INIT_FROM_BOARD_CONF (&status, devBoardConf, *pUARTBoardConf);
        if (status != eSTATUS_SUCCESS) {
            goto error;
        }
    }

    if (LoggerAddSink (SerialDebugSink) != eSTATUS_SUCCESS) {
        goto error;
    }

    g_SerialDebug.isInitialized = true;
    return eSTATUS_SUCCESS;
error:
    memset (&g_SerialDebug, 0, sizeof (g_SerialDebug));
    return eSTATUS_FAILURE;
}
