#include "log/logger.h"
#include "common.h"
#include "conf.h"
#include "hal.h"
#include "log/format.h"
#include "mem/mem.h"
#include "mem/ring_buff.h"
#include "periphs/uart.h"
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
#define TEMP_BUFFER_SIZE     512U
#define UART_LOGGER_HANDLE   egHandleUSART_1
#define UART_LOGGER_INSTANCE USART1

// NOLINTBEGIN
static uint8_t ga_TempReadBuffer[TEMP_BUFFER_SIZE] = { 0 };
static RingBuff volatile* gp_CM4_RingBuf           = NULL;
static RingBuff volatile* gp_CM7_RingBuf           = NULL;
// NOLINTEND

static eSTATUS_t LoggerSyncUARTTaskHandler (DefaultTask const* pTask);
static eSTATUS_t LoggerWriteToUART (RingBuff volatile* pRingBuf, uint32_t totalLen);
static RingBuff volatile* LoggerGetMyRingBuf (void);
static RingBuff volatile* LoggerGetOtherRingBuf (void);
static void LoggerWriteChar_Blocking (char ch);
static void LoggerWriteChar_NonBlocking (char ch);
static eSTATUS_t LoggerUARTInit (void);
static void LoggerPutChar (void* p, char ch);

static eSTATUS_t LoggerSyncUARTTaskHandler (DefaultTask const* pTask) {
    // Write the other core's ring buffer out to the UART
    if (PRIMARY_LOGGER_IS_ME () == TRUE) {
        SyncTaskUartOut const* pSyncTaskUartOut = (SyncTaskUartOut const*)pTask;
        return LoggerWriteToUART (
        LoggerGetOtherRingBuf (), pSyncTaskUartOut->len);
    }
    return eSTATUS_SUCCESS;
}

static eSTATUS_t LoggerWriteToUART (RingBuff volatile* pRingBuf, uint32_t totalLen) {

    if (RingBuffIsValid (pRingBuf) != TRUE || UART_LOGGER_HANDLE.Instance == NULL) {
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
        if (HAL_UART_Transmit (&UART_LOGGER_HANDLE, ga_TempReadBuffer, bytesRead, 1000) != HAL_OK) {
            return eSTATUS_FAILURE;
        }
        nTotalBytes -= bytesRead;
    }

    return eSTATUS_SUCCESS;
}

/*
 * Returns the CURRENT core's ring buffer
 */
static RingBuff volatile* LoggerGetMyRingBuf (void) {
    return (HAL_GetCurrentCPUID () == CM7_CPUID) ? gp_CM7_RingBuf : gp_CM4_RingBuf;
}

/*
 * Returns the OTHER core's ring buffer
 */
static RingBuff volatile* LoggerGetOtherRingBuf (void) {
    return (HAL_GetCurrentCPUID () == CM7_CPUID) ? gp_CM4_RingBuf : gp_CM7_RingBuf;
}

static void LoggerWriteChar_Blocking (char ch) {

    RingBuff volatile* pMyRingBuf = LoggerGetMyRingBuf ();
    /*
     * If the char will cause an overwrite in the ring buffer:
     *   Primary Logger = write what is currently in ring buffer to uart
     *   Secondary Logger = wait until primary logger writes my ring buffer out to the uart and makes more space.
     */
    if (RingBuffGetFree (pMyRingBuf) == 0U) {
        if (PRIMARY_LOGGER_IS_ME () == TRUE) {
            LoggerWriteToUART (pMyRingBuf, RingBuffGetFull (pMyRingBuf));
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

    RingBuff volatile* pMyRingBuf = LoggerGetMyRingBuf ();
    RingBuffWrite (pMyRingBuf, (void*)&ch, 1);

    if ((char)ch == '\n') {
        if (PRIMARY_LOGGER_IS_ME () == TRUE) {
            LoggerWriteToUART (pMyRingBuf, RingBuffGetFull (pMyRingBuf));
        } else {
            SyncNotifyTaskUartOut (RingBuffGetFull (pMyRingBuf));
        }
    }
}

/*
 * The GPIO setup happens in uart.c
 */
static eSTATUS_t LoggerUARTInit (void) {

#ifndef UNIT_TEST
    // puart->Init.BaudRate = 115200;
    UART_LOGGER_HANDLE.Instance            = UART_LOGGER_INSTANCE;
    UART_LOGGER_HANDLE.Init.BaudRate       = 230400; // 115200; //
    UART_LOGGER_HANDLE.Init.WordLength     = UART_WORDLENGTH_8B;
    UART_LOGGER_HANDLE.Init.StopBits       = UART_STOPBITS_1;
    UART_LOGGER_HANDLE.Init.Parity         = UART_PARITY_NONE;
    UART_LOGGER_HANDLE.Init.Mode           = UART_MODE_TX_RX;
    UART_LOGGER_HANDLE.Init.HwFlowCtl      = UART_HWCONTROL_NONE;
    UART_LOGGER_HANDLE.Init.OverSampling   = UART_OVERSAMPLING_16;
    UART_LOGGER_HANDLE.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    UART_LOGGER_HANDLE.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    UART_LOGGER_HANDLE.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

    /*
     * HAL_UART_Init calls HAL_UART_MspInit in uart.c
     */
    if (HAL_UART_Init (&UART_LOGGER_HANDLE) != HAL_OK) {
        return eSTATUS_FAILURE;
    }

    if (HAL_UARTEx_SetTxFifoThreshold (&UART_LOGGER_HANDLE, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK) {
        return eSTATUS_FAILURE;
    }

    if (HAL_UARTEx_SetRxFifoThreshold (&UART_LOGGER_HANDLE, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK) {
        return eSTATUS_FAILURE;
    }

    if (HAL_UARTEx_DisableFifoMode (&UART_LOGGER_HANDLE) != HAL_OK) {
        return eSTATUS_FAILURE;
    }
#endif // UNIT_TEST
    return eSTATUS_SUCCESS;
}

/*
 * Used by printf impl in format.c
 */
static void LoggerPutChar (void* p, char ch) {
    LOGGER_WRITE_CHAR ((char)ch);
}

// PUTCHAR_PROTOTYPE {
//     LOGGER_WRITE_CHAR ((char)ch);
//     return ch;
// }

eSTATUS_t LoggerInit (void) {

    init_printf (NULL, LoggerPutChar);

    if (PRIMARY_LOGGER_IS_ME () == TRUE) {
        if (LoggerUARTInit () != eSTATUS_SUCCESS) {
            return eSTATUS_FAILURE;
        }
    }

    // Let both cores register this task handler only the primary logger
    // core will actually write to the UART
    if (SyncRegisterHandler (eSYNC_TASKID_UART_OUT, LoggerSyncUARTTaskHandler) != eSTATUS_SUCCESS) {
        return eSTATUS_FAILURE;
    }

    /*
     * Local variables are not shared among the cores.
     * So each ring buffer pointer needs to be inited for each core
     */
    gp_CM4_RingBuf =
    RingBuffCreate ((void*)MEM_SHARED_CM4_UART_RINGBUFF_START, MEM_SHARED_CM4_UART_RINGBUFF_TOTAL_LEN);
    gp_CM7_RingBuf =
    RingBuffCreate ((void*)MEM_SHARED_CM7_UART_RINGBUFF_START, MEM_SHARED_CM7_UART_RINGBUFF_TOTAL_LEN);
    if (gp_CM4_RingBuf == NULL || gp_CM7_RingBuf == NULL) {
        return eSTATUS_FAILURE;
    }
    return eSTATUS_SUCCESS;
}

UART_HandleTypeDef* LoggerGetUARTHandle (void) {
    if (PRIMARY_LOGGER_IS_ME () == TRUE) {
        return &UART_LOGGER_HANDLE;
    }
    return NULL;
}
