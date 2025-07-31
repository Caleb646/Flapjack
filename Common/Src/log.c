#include <stdio.h>
#include <string.h>

#include "hal.h"
#include "log.h"
#include "mem/mem.h"
#include "mem/ring_buff.h"
#include "periphs/uart.h"
#include "sync.h"



#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar (int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc (int ch, FILE* f)
#endif

#define UART_LOGGER_HANDLE   egHandleUSART_1
#define UART_LOGGER_INSTANCE USART1
#define PRIMARY_LOGGER_ROLE  CM4_CPUID

// NOLINTBEGIN
static uint8_t gaBuffer[512]           = { 0 };
static RingBuff volatile* gpCM4RingBuf = NULL;
static RingBuff volatile* gpCM7RingBuf = NULL;
// NOLINTEND

static eSTATUS_t LoggerSyncUARTTaskHandler (DefaultTask const* pTask);
static eSTATUS_t LoggerWriteToUART (RingBuff volatile* pRingBuf, int32_t totalLen);

static eSTATUS_t LoggerWriteToUART (RingBuff volatile* pRingBuf, int32_t totalLen) {

    if (pRingBuf == NULL || UART_LOGGER_HANDLE.Instance == NULL) {
        return eSTATUS_FAILURE;
    }

    if (totalLen <= 0 || totalLen > sizeof (gaBuffer)) {
        return eSTATUS_FAILURE;
    }
    /* Read totalLen bytes from ringbuffer and this will include bytes
     * that overflowed (wrapped around to the beginning of the buffer)
     */
    uint32_t bytesRead = RingBuffRead (pRingBuf, (void*)gaBuffer, totalLen);
    if (HAL_UART_Transmit (&UART_LOGGER_HANDLE, gaBuffer, bytesRead, 1000) != HAL_OK) {
        return eSTATUS_FAILURE;
    }
    return eSTATUS_SUCCESS;
}

static eSTATUS_t LoggerSyncUARTTaskHandler (DefaultTask const* pTask) {
    // Write the other core's ring buffer out to the UART
    if (HAL_GetCurrentCPUID () == PRIMARY_LOGGER_ROLE) {
        SyncTaskUartOut const* pSyncTaskUartOut = (SyncTaskUartOut const*)pTask;
        RingBuff volatile* pMyRingBuf =
        (HAL_GetCurrentCPUID () == CM7_CPUID) ? gpCM4RingBuf : gpCM7RingBuf;
        return LoggerWriteToUART (pMyRingBuf, pSyncTaskUartOut->len);
    }
    return eSTATUS_SUCCESS;
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

PUTCHAR_PROTOTYPE {

    RingBuff volatile* pMyRingBuf =
    (HAL_GetCurrentCPUID () == CM7_CPUID) ? gpCM7RingBuf : gpCM4RingBuf;

    if (RingBuffIsValid (pMyRingBuf) != TRUE) {
        return ch;
    }

    RingBuffWrite (pMyRingBuf, (void*)&ch, 1);
    if ((char)ch == '\n') {
        if (HAL_GetCurrentCPUID () == PRIMARY_LOGGER_ROLE) {
            LoggerWriteToUART (pMyRingBuf, RingBuffGetFull (pMyRingBuf));
        } else {
            SyncNotifyTaskUartOut (RingBuffGetFull (pMyRingBuf));
        }
    }
    return ch;
}

eSTATUS_t LoggerInit (void) {

    if (HAL_GetCurrentCPUID () == PRIMARY_LOGGER_ROLE) {
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
    gpCM4RingBuf =
    RingBuffCreate ((void*)MEM_SHARED_CM4_UART_RINGBUFF_START, MEM_SHARED_CM4_UART_RINGBUFF_TOTAL_LEN);
    gpCM7RingBuf =
    RingBuffCreate ((void*)MEM_SHARED_CM7_UART_RINGBUFF_START, MEM_SHARED_CM7_UART_RINGBUFF_TOTAL_LEN);
    if (gpCM4RingBuf == NULL || gpCM7RingBuf == NULL) {
        return eSTATUS_FAILURE;
    }
    return eSTATUS_SUCCESS;
}

UART_HandleTypeDef* LoggerGetUARTHandle (void) {
    if (HAL_GetCurrentCPUID () == PRIMARY_LOGGER_ROLE) {
        return &UART_LOGGER_HANDLE;
    }
    return NULL;
}