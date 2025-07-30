#include <stdio.h>
#include <string.h>

#include "hal.h"
#include "log.h"
#include "mem/mem.h"
#include "mem/ring_buff.h"
#include "sync.h"


#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar (int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc (int ch, FILE* f)
#endif

// NOLINTBEGIN
static uint8_t gaBuffer[512]           = { 0 };
static RingBuff volatile* gpCM4RingBuf = NULL;
static RingBuff volatile* gpCM7RingBuf = NULL;
static UART_HandleTypeDef gUART        = { 0 };
static uint32_t gLoggerUARTRole        = 0; // Default to invalid
// NOLINTEND

static STATUS_TYPE LoggerSyncUARTTaskHandler (DefaultTask const* pTask);
static STATUS_TYPE LoggerWriteToUART (RingBuff volatile* pRingBuf, int32_t totalLen);
static STATUS_TYPE
LoggerUARTInit (USART_TypeDef* pUartInstance, UART_HandleTypeDef* pOutUartHandle);
static STATUS_TYPE LoggerSharedInit (void);

static STATUS_TYPE LoggerWriteToUART (RingBuff volatile* pRingBuf, int32_t totalLen) {

    if (pRingBuf == NULL || gUART.Instance == NULL || totalLen <= 0 || totalLen > sizeof (gaBuffer)) {
        return eSTATUS_FAILURE;
    }
    /* Read totalLen bytes from ringbuffer and this will include bytes
     * that overflowed (wrapped around to the beginning of the buffer)
     */
    uint32_t bytesRead = RingBuffRead (pRingBuf, (void*)gaBuffer, totalLen);
    if (HAL_UART_Transmit (&gUART, gaBuffer, bytesRead, 1000) != HAL_OK) {
        return eSTATUS_FAILURE;
    }
    return eSTATUS_SUCCESS;
}

static STATUS_TYPE LoggerSyncUARTTaskHandler (DefaultTask const* pTask) {
    // Write the other core's ring buffer out to the UART
    if (HAL_GetCurrentCPUID () == gLoggerUARTRole) {
        SyncTaskUartOut const* pSyncTaskUartOut = (SyncTaskUartOut const*)pTask;
        if (gLoggerUARTRole == CM7_CPUID) {
            LoggerWriteToUART (gpCM4RingBuf, pSyncTaskUartOut->len);
        } else {
            LoggerWriteToUART (gpCM7RingBuf, pSyncTaskUartOut->len);
        }
    }
    return eSTATUS_SUCCESS;
}

/*
 * The GPIO setup happens in uart.c
 */
static STATUS_TYPE
LoggerUARTInit (USART_TypeDef* pUartInstance, UART_HandleTypeDef* pOutUartHandle) {
    if (pUartInstance == NULL || pOutUartHandle == NULL) {
        return eSTATUS_FAILURE;
    }

#ifndef UNIT_TEST
    // puart->Init.BaudRate = 115200;
    pOutUartHandle->Instance            = pUartInstance;
    pOutUartHandle->Init.BaudRate       = 230400;
    pOutUartHandle->Init.WordLength     = UART_WORDLENGTH_8B;
    pOutUartHandle->Init.StopBits       = UART_STOPBITS_1;
    pOutUartHandle->Init.Parity         = UART_PARITY_NONE;
    pOutUartHandle->Init.Mode           = UART_MODE_TX_RX;
    pOutUartHandle->Init.HwFlowCtl      = UART_HWCONTROL_NONE;
    pOutUartHandle->Init.OverSampling   = UART_OVERSAMPLING_16;
    pOutUartHandle->Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    pOutUartHandle->Init.ClockPrescaler = UART_PRESCALER_DIV1;
    pOutUartHandle->AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

    /*
     * HAL_UART_Init calls HAL_UART_MspInit in uart.c
     */
    if (HAL_UART_Init (pOutUartHandle) != HAL_OK) {
        return eSTATUS_FAILURE;
    }

    if (HAL_UARTEx_SetTxFifoThreshold (pOutUartHandle, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK) {
        return eSTATUS_FAILURE;
    }

    if (HAL_UARTEx_SetRxFifoThreshold (pOutUartHandle, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK) {
        return eSTATUS_FAILURE;
    }

    if (HAL_UARTEx_DisableFifoMode (pOutUartHandle) != HAL_OK) {
        return eSTATUS_FAILURE;
    }
#endif // UNIT_TEST
    return eSTATUS_SUCCESS;
}

static STATUS_TYPE LoggerSharedInit (void) {
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

PUTCHAR_PROTOTYPE {

    RingBuff volatile* pMyRingBuf =
    (HAL_GetCurrentCPUID () == CM7_CPUID) ? gpCM7RingBuf : gpCM4RingBuf;

    if (RingBuffIsValid (pMyRingBuf) != TRUE) {
        return ch;
    }

    if (gLoggerUARTRole != CM7_CPUID && gLoggerUARTRole != CM4_CPUID) {
        return ch;
    }

    RingBuffWrite (pMyRingBuf, (void*)&ch, 1);
    if ((char)ch == '\n') {
        if (HAL_GetCurrentCPUID () == gLoggerUARTRole) {
            LoggerWriteToUART (pMyRingBuf, RingBuffGetFull (pMyRingBuf));
        } else {
            SyncNotifyTaskUartOut (RingBuffGetFull (pMyRingBuf));
        }
    }
    return ch;
}

/*
 * This function just initializes the ring buffers and registers the UART task handler.
 * The core that called this function will NOT directly write to the UART. It will send
 * a notification to the other core to write to the UART.
 */
STATUS_TYPE LoggerSecondaryInit (void) {

    if (gLoggerUARTRole == HAL_GetCurrentCPUID () || LoggerSharedInit () != eSTATUS_SUCCESS) {
        return eSTATUS_FAILURE;
    }
    if (SyncRegisterHandler (eSYNC_TASKID_UART_OUT, LoggerSyncUARTTaskHandler) != eSTATUS_SUCCESS) {
        return eSTATUS_FAILURE;
    }
    return eSTATUS_SUCCESS;
}

/*
 * This function inits the UART. The core that initializes the UART will be the one that writes to it.
 * The other core will only write to the ring buffer.
 */
STATUS_TYPE LoggerPrimaryInit (USART_TypeDef* pUartInstance, UART_HandleTypeDef* pOutUartHandle) {

    if (pUartInstance == NULL || pOutUartHandle == NULL) {
        return eSTATUS_FAILURE;
    }

    gLoggerUARTRole = HAL_GetCurrentCPUID ();
    if (LoggerUARTInit (pUartInstance, pOutUartHandle) != eSTATUS_SUCCESS) {
        return eSTATUS_FAILURE;
    }
    if (LoggerSharedInit () != eSTATUS_SUCCESS) {
        return eSTATUS_FAILURE;
    }
    return eSTATUS_SUCCESS;
}