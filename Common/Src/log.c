#include <stdio.h>
#include <string.h>

#include "hal.h"
#include "log.h"
#include "mem/mem.h"
#include "mem/ring_buff.h"
#include "sync/mailbox.h"
#include "sync/sync.h"


#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar (int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc (int ch, FILE* f)
#endif

// NOLINTBEGIN
static RingBuff volatile* gpCM4RingBuf = NULL;
static RingBuff volatile* gpCM7RingBuf = NULL;
static UART_HandleTypeDef gUART        = { 0 };
static uint32_t gLoggerUARTRole        = 0; // Default to invalid
// NOLINTEND

static STATUS_TYPE LoggerSyncUARTTaskHandler (void);
static STATUS_TYPE LoggerWriteToUART (RingBuff volatile* pRingBuf);
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
            LoggerWriteToUART (pMyRingBuf);
        } else {
            uint32_t taskID = SYNC_TASKID_UART_OUT;
            uint32_t mailboxID =
            (gLoggerUARTRole == CM7_CPUID) ? MAILBOX_CM7_ID : MAILBOX_CM4_ID;
            SyncMailBoxWriteNotify (mailboxID, (uint8_t*)&taskID, sizeof (uint32_t));
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

    status = SyncRegisterHandler (LoggerSyncUARTTaskHandler, SYNC_TASKID_UART_OUT);
    if (status != eSTATUS_SUCCESS) {
        return status;
    }

    return status;
}

static STATUS_TYPE LoggerWriteToUART (RingBuff volatile* pRingBuf) {
    uint32_t len         = 0;
    void* pBufToTransmit = NULL;

    if (pRingBuf == NULL || gUART.Instance == NULL) {
        return eSTATUS_FAILURE;
    }

send:
    len = RingBuffGetLinearBlockReadLength (pRingBuf);
    if (len > 0) {
        pBufToTransmit = RingBuffGetLinearBlockReadAddress (pRingBuf);
        if (HAL_UART_Transmit (&gUART, pBufToTransmit, len, 1000) != HAL_OK) {
            return eSTATUS_FAILURE;
        }
        RingBuffSkip (pRingBuf, len);
        /* Check for anything in the overflow buffer */
        goto send;
    }
    return eSTATUS_SUCCESS;
}

static STATUS_TYPE LoggerSyncUARTTaskHandler (void) {
    // Write the other core's ring buffer out to the UART
    if (HAL_GetCurrentCPUID () == gLoggerUARTRole) {
        if (gLoggerUARTRole == CM7_CPUID) {
            LoggerWriteToUART (gpCM4RingBuf);
        } else {
            LoggerWriteToUART (gpCM7RingBuf);
        }
    }
    return eSTATUS_SUCCESS;
}

static STATUS_TYPE LoggerUARTInit (UART_HandleTypeDef* huart1, USART_TypeDef* pUARTInstance) {
    if (huart1 == NULL || pUARTInstance == NULL) {
        return eSTATUS_FAILURE;
    }

#ifndef UNIT_TEST
    huart1->Instance                    = pUARTInstance;
    huart1->Init.BaudRate               = 115200;
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

// void LogStackTrace (void) {

//     LOG_ERROR ("=== STACK TRACE BEGIN ===");

//     // Use GCC built-in functions to get current addresses
//     void* current_pc = __builtin_return_address (0);
//     void* caller_pc  = 0;

//     LOG_ERROR ("Current function return address: 0x%08lX", (unsigned long)current_pc);

//     // Try to get caller addresses using GCC built-ins
//     // Note: This may not work in all optimization levels
//     int frame_count      = 0;
//     const int max_frames = 8;

//     for (int i = 0; i < max_frames; i++) {
//         void* return_addr = 0;

//         // Use compiler built-in to get return address at different
//         levels switch (i) { case 0: return_addr =
//         __builtin_return_address (0); break; case 1:
//             // Check if we can safely get level 1
//             if (__builtin_frame_address (1) != 0) {
//                 return_addr = __builtin_return_address (1);
//             }
//             break;
//         case 2:
//             if (__builtin_frame_address (2) != 0) {
//                 return_addr = __builtin_return_address (2);
//             }
//             break;
//         case 3:
//             if (__builtin_frame_address (3) != 0) {
//                 return_addr = __builtin_return_address (3);
//             }
//             break;
//         default:
//             // For higher levels, we'll break as it gets unreliable
//             break;
//         }

//         if (return_addr == 0) {
//             break;
//         }

//         // Check if this looks like a valid code address
//         unsigned long addr = (unsigned long)return_addr;
//         if (addr >= 0x08000000 && addr <= 0x08FFFFFF) {
//             LOG_ERROR ("Frame %d: Return Address = 0x%08lX",
//             frame_count, addr); frame_count++;
//         }
//     }

//     // Additional debugging information
//     LOG_ERROR ("Stack trace depth: %d frames", frame_count);

//     // Get current stack pointer using a local variable address
//     volatile uint32_t stack_marker = 0xDEADBEEF;
//     LOG_ERROR ("Approximate stack pointer: 0x%08lX", (unsigned long)&stack_marker);

//     // Log memory regions for reference
//     LOG_ERROR ("Flash memory range: 0x08000000 - 0x08FFFFFF");
//     LOG_ERROR ("RAM memory range: 0x20000000 - 0x2007FFFF");

//     LOG_ERROR ("=== STACK TRACE END ===");

//     // Keep the breakpoint for debugging
//     __BKPT (2);
// }
