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
static uint8_t gpBuffer[512]           = { 0 };
static RingBuff volatile* gpCM4RingBuf = NULL;
static RingBuff volatile* gpCM7RingBuf = NULL;
static UART_HandleTypeDef gUART        = { 0 };
static uint32_t gLoggerUARTRole        = 0; // Default to invalid
// NOLINTEND

static STATUS_TYPE LoggerSyncUARTTaskHandler (DefaultTask const* pTask);
static STATUS_TYPE LoggerWriteToUART (RingBuff volatile* pRingBuf, int32_t totalLen);
static STATUS_TYPE LoggerUARTInit (UART_HandleTypeDef* huart1, USART_TypeDef* pUARTInstance);

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

STATUS_TYPE LoggerInit (USART_TypeDef* pUARTInstance, UART_HandleTypeDef** pOutUARTHandle) {

    STATUS_TYPE status = eSTATUS_SUCCESS;
    /*
     * If pUARTInstance is NULL, the other core will be responsible for initializing the UART.
     */
    if (pUARTInstance == NULL) {
        if (HAL_GetCurrentCPUID () == CM7_CPUID) {
            gLoggerUARTRole = CM4_CPUID;
        } else {
            gLoggerUARTRole = CM7_CPUID;
        }
    } else {
        /*
         * If pUARTInstance is NOT NULL, then this core is responsible for initializing the UART.
         */
        if (HAL_GetCurrentCPUID () == CM7_CPUID) {
            gLoggerUARTRole = CM7_CPUID;
        } else {
            gLoggerUARTRole = CM4_CPUID;
        }

        status = LoggerUARTInit (&gUART, pUARTInstance);
        if (status != eSTATUS_SUCCESS) {
            return status;
        }

        if (pOutUARTHandle != NULL) {
            *pOutUARTHandle = &gUART;
        }
    }

    ASSERT (gLoggerUARTRole == CM7_CPUID || gLoggerUARTRole == CM4_CPUID);
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

    status = SyncRegisterHandler (eSYNC_TASKID_UART_OUT, LoggerSyncUARTTaskHandler);
    if (status != eSTATUS_SUCCESS) {
        return status;
    }

    return status;
}

static STATUS_TYPE LoggerWriteToUART (RingBuff volatile* pRingBuf, int32_t totalLen) {

    if (pRingBuf == NULL || gUART.Instance == NULL || totalLen <= 0 || totalLen > sizeof (gpBuffer)) {
        return eSTATUS_FAILURE;
    }
    /* Read totalLen bytes from ringbuffer and this will include bytes
     * that overflowed (wrapped around to the beginning of the buffer)
     */
    uint32_t bytesRead = RingBuffRead (pRingBuf, (void*)gpBuffer, totalLen);
    if (HAL_UART_Transmit (&gUART, gpBuffer, bytesRead, 1000) != HAL_OK) {
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

static STATUS_TYPE LoggerUARTInit (UART_HandleTypeDef* huart1, USART_TypeDef* pUARTInstance) {
    if (huart1 == NULL || pUARTInstance == NULL) {
        return eSTATUS_FAILURE;
    }

#ifndef UNIT_TEST
    huart1->Instance = pUARTInstance;
    // huart1->Init.BaudRate = 115200;
    huart1->Init.BaudRate               = 230400;
    huart1->Init.WordLength             = UART_WORDLENGTH_8B;
    huart1->Init.StopBits               = UART_STOPBITS_1;
    huart1->Init.Parity                 = UART_PARITY_NONE;
    huart1->Init.Mode                   = UART_MODE_TX_RX;
    huart1->Init.HwFlowCtl              = UART_HWCONTROL_NONE;
    huart1->Init.OverSampling           = UART_OVERSAMPLING_16;
    huart1->Init.OneBitSampling         = UART_ONE_BIT_SAMPLE_DISABLE;
    huart1->Init.ClockPrescaler         = UART_PRESCALER_DIV1;
    huart1->AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

    /*
     * HAL_UART_Init calls HAL_UART_MspInit in uart.c
     */
    if (HAL_UART_Init (huart1) != HAL_OK) {
        return eSTATUS_FAILURE;
    }

    if (HAL_UARTEx_SetTxFifoThreshold (huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK) {
        return eSTATUS_FAILURE;
    }

    if (HAL_UARTEx_SetRxFifoThreshold (huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK) {
        return eSTATUS_FAILURE;
    }

    if (HAL_UARTEx_DisableFifoMode (huart1) != HAL_OK) {
        return eSTATUS_FAILURE;
    }
#endif // UNIT_TEST
    return eSTATUS_SUCCESS;
}