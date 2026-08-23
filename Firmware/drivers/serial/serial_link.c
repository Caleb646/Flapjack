#include "hal.h"
#include "target.h"

#include "core/core.h"

#include "common/ring_buff.h"

#include "drivers/serial/serial_link.h"
#include "drivers/serial/uart.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "stream_buffer.h"

#include <string.h>

/* One baud for every build - the board owns it. It used to be overridden under
 * SIM_HIL, which meant the same number lived in two places and could drift. */
#define SERIAL_LINK_BAUD BRD_GET_BAUD_RATE (SERIAL_LINK)

#define SERIAL_MAGIC0    0xAAU
#define SERIAL_MAGIC1    0x55U

#define SERIAL_RX_STREAM 512U
#define SERIAL_TX_TEXT   2048U

/*
 * Text is written in small chunks, releasing the TX mutex between each one.
 * This is the bound on how long a log line can make an actuator frame wait:
 * draining a full 2 KB block under the mutex would block the control task for
 * ~44 ms at 460800. 32 bytes is ~0.7 ms, comfortably inside the 2.5 ms control
 * period at 400 Hz.
 */
#define SERIAL_TX_TEXT_CHUNK 32U

#if BRD_IS_ENABLED(SERIAL_LINK)

static UartPort_t s_port;
static StreamBufferHandle_t s_rxStream;
static SemaphoreHandle_t s_txMutex;   // owns the UART: frames and text chunks
static SemaphoreHandle_t s_textMutex; // owns the text ring's write side
static TaskHandle_t s_txTask;

static uint8_t s_textStorage[SERIAL_TX_TEXT];
static RingBuff s_textRing;

static SerialLink_Handler_t s_handlers[SERIAL_LINK_MAX_MSG_ID];
static volatile uint32_t s_droppedBytes;

/* CRC8 (poly 0x07, init 0x00) over (msg_id, len, payload). Mirrors the PC. */
static uint8_t SerialCrc8 (uint8_t const* pData, uint32_t len) {
    uint8_t crc = 0U;
    for (uint32_t i = 0; i < len; ++i) {
        crc ^= pData[i];
        for (uint8_t b = 0; b < 8U; ++b) {
            crc = (crc & 0x80U) ? (uint8_t)((crc << 1) ^ 0x07U) : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

static void SerialLink_RxIsr (uint8_t const* pData, uint32_t len) {
    BaseType_t woken = pdFALSE;
    (void)xStreamBufferSendFromISR (s_rxStream, pData, len, &woken);
    portYIELD_FROM_ISR (woken);
}

/*
 * Before vTaskStartScheduler() there is no TX task to drain the rings, so the
 * send paths fall through to a direct blocking write. That covers every LOG_*
 * in main()'s init sequence with no special casing at the call sites.
 */
static bool SerialLink_SchedulerRunning (void) {
    return xTaskGetSchedulerState () == taskSCHEDULER_RUNNING;
}

static void SerialLink_NotifyTx (void) {
    if (s_txTask != NULL) {
        (void)xTaskNotifyGive (s_txTask);
    }
}

static void SerialLink_LogSink (uint8_t const* pData, uint32_t len) {
    SerialLink_SendText (pData, len);
    // TODO: at some point this should be addressed on whether to keep this or not
    // for (uint32_t i = 0; i < len; ++i) {
    //     ITM_SendChar (pData[i]);
    // }
}

eSTATUS_t SerialLink_Init (void) {

    s_rxStream  = xStreamBufferCreate (SERIAL_RX_STREAM, 1U);
    s_textMutex = xSemaphoreCreateMutex ();
    s_txMutex   = xSemaphoreCreateMutex ();

    /* Undo the BASEPRI mask the three creates above leave behind - see the
     * comment in Spi_InitSystem() for why a pre-scheduler critical section
     * never unmasks on exit. This function only survived without it by running
     * after the last pre-scheduler HAL_Delay; that is not a property worth
     * depending on, since the next thing added to main() can silently break it.
     * Before the check below, so a failed create still returns with interrupts
     * on and CriticalErrorHandler() can use the tick. */
    portENABLE_INTERRUPTS ();

    if (!s_rxStream || !s_textMutex || !s_txMutex) {
        return eSTATUS_FAILURE;
    }

    if (!RingBuffInit (s_textStorage, sizeof (s_textStorage), &s_textRing)) {
        return eSTATUS_FAILURE;
    }

    s_port.cfg.id          = BRD_GET_UART_ID (SERIAL_LINK);
    s_port.cfg.baudRate    = SERIAL_LINK_BAUD;
    s_port.cfg.rxCallback  = SerialLink_RxIsr;
    s_port.cfg.irqPriority = 6U;
    if (STATUS_FAIL (UartPort_Init (&s_port))) {
        return eSTATUS_FAILURE;
    }

    return LoggerAddSink (SerialLink_LogSink);
}

eSTATUS_t SerialLink_RegisterHandler (uint8_t msgId, SerialLink_Handler_t handler) {

    if (msgId >= SERIAL_LINK_MAX_MSG_ID || handler == NULL) {
        return eSTATUS_FAILURE;
    }
    s_handlers[msgId] = handler;
    return eSTATUS_SUCCESS;
}

uint32_t SerialLink_GetDroppedBytes (void) {
    return s_droppedBytes;
}

/* --- TX -------------------------------------------------------------------- */

void SerialLink_SendText (uint8_t const* pData, uint32_t len) {

    if (!pData || !len) {
        return;
    }

    if (!SerialLink_SchedulerRunning ()) {
        (void)UartPort_Write (&s_port, pData, len);
        return;
    }

    /*
     * Timeout 0: a logging task must never block on the link. A large write
     * (LoggerWriteToSinks can hand us a full 4 KB linear block) is why this is
     * a mutex rather than a critical section - masking interrupts that long
     * would overrun the UART RX, which has no FIFO.
     */
    if (xSemaphoreTake (s_textMutex, 0U) != pdTRUE) {
        s_droppedBytes += len;
        return;
    }
    uint32_t written = (uint32_t)RingBuffWrite (&s_textRing, pData, len);
    (void)xSemaphoreGive (s_textMutex);

    if (written < len) {
        s_droppedBytes += (len - written);
    }
    SerialLink_NotifyTx ();
}

eSTATUS_t SerialLink_SendFrame (uint8_t msgId, uint8_t const* pPayload, uint8_t len) {

    if (len > SERIAL_LINK_MAX_PAYLOAD || (!pPayload && len)) {
        return eSTATUS_FAILURE;
    }

    uint8_t frame[SERIAL_LINK_MAX_FRAME];
    frame[0] = SERIAL_MAGIC0;
    frame[1] = SERIAL_MAGIC1;
    frame[2] = msgId;
    frame[3] = len;
    if (len) {
        memcpy (&frame[4], pPayload, len);
    }
    frame[4 + len]  = SerialCrc8 (&frame[2], (uint32_t)len + 2U); /* over id,len,payload */
    uint32_t total  = (uint32_t)len + 5U;

    if (!SerialLink_SchedulerRunning ()) {
        return UartPort_Write (&s_port, frame, total);
    }

    /*
     * Frames go out synchronously, not through a queue. This keeps actuator
     * timing byte-for-byte identical to what the sim link did before this
     * module existed, and frame traffic is rate-limited by its producers
     * anyway (~13 kB/s, 29 % of the link), so a queue would buy nothing but
     * latency. Only text, which is unbounded and can afford to wait, is
     * buffered.
     *
     * The mutex is what makes sharing safe: it serialises against the other
     * frame producer and against the TX task's text chunks, so no frame is
     * ever split. Priority inheritance bounds the wait, and the text chunk
     * size bounds how long the TX task can hold it.
     */
    if (xSemaphoreTake (s_txMutex, portMAX_DELAY) != pdTRUE) {
        return eSTATUS_FAILURE;
    }
    eSTATUS_t status = UartPort_Write (&s_port, frame, total);
    (void)xSemaphoreGive (s_txMutex);
    return status;
}

/*
 * Write one chunk of pending text, taking the UART mutex for just that chunk so
 * a frame producer can cut in between chunks. Returns false when the ring is
 * empty. `lock` is false only on the panic path, where the scheduler may be
 * gone and taking a mutex would hang.
 */
static bool SerialLink_DrainTextChunk (bool lock) {

    uint32_t len = (uint32_t)RingBuffGetLinearBlockReadLength (&s_textRing);
    if (len == 0U) {
        return false;
    }
    if (len > SERIAL_TX_TEXT_CHUNK) {
        len = SERIAL_TX_TEXT_CHUNK;
    }
    uint8_t const* pData = (uint8_t const*)RingBuffGetLinearBlockReadAddress (&s_textRing);

    if (lock) {
        if (xSemaphoreTake (s_txMutex, portMAX_DELAY) != pdTRUE) {
            return false;
        }
        (void)UartPort_Write (&s_port, pData, len);
        (void)xSemaphoreGive (s_txMutex);
    } else {
        (void)UartPort_Write (&s_port, pData, len);
    }
    (void)RingBuffSkip (&s_textRing, len);
    return true;
}

/*
 * Blocks until there is buffered log text, then writes it out. Frames do NOT
 * come through here - they are written synchronously by their producer (see
 * SerialLink_SendFrame), so this path only ever carries traffic that can
 * afford to wait.
 *
 * MUST NOT LOG. The logger feeds the ring this drains, so a LOG_* from here is
 * a self-deadlock - the same discipline as vApplicationStackOverflowHook.
 */
void SerialLink_TxUpdate (void) {

    /* The caller is the TX task; record it so producers can wake us. */
    if (s_txTask == NULL) {
        s_txTask = xTaskGetCurrentTaskHandle ();
    }

    (void)ulTaskNotifyTake (pdTRUE, portMAX_DELAY);

    /*
     * Drain the other core's log ring buffer through the sinks. No-op when the
     * queue is empty, which is always the case in single-core builds.
     */
    (void)SyncProcessTasks ();

    while (SerialLink_DrainTextChunk (true)) {
    }
}

/*
 * Panic path: flush synchronously so a halt does not swallow the diagnosis that
 * explains it. Overrides the weak no-op in core_shared.c, which lets core/ stay
 * free of any dependency on drivers/. Frames need no flush - they were already
 * written synchronously.
 */
void CriticalErrorFlushHook (void) {

    while (SerialLink_DrainTextChunk (false)) {
    }
}

/* --- RX -------------------------------------------------------------------- */

typedef enum { ST_MAGIC0, ST_MAGIC1, ST_ID, ST_LEN, ST_PAYLOAD, ST_CRC } RxState_t;

/*
 * Deframer state. File scope rather than local because the loop that drives it
 * lives in tasks/serial_link/ now; there is exactly one consumer of the RX
 * stream, so a single instance is correct.
 */
static RxState_t s_rxState = ST_MAGIC0;
static uint8_t s_rxId, s_rxLen, s_rxIdx;
static uint8_t s_rxPayload[SERIAL_LINK_MAX_PAYLOAD];

/*
 * Blocks for one inbound byte and advances the deframer. On a complete frame
 * whose CRC checks out, calls the handler registered for its msg id.
 */
void SerialLink_RxUpdate (void) {

    uint8_t byte;
    if (xStreamBufferReceive (s_rxStream, &byte, 1U, portMAX_DELAY) != 1U) {
        return;
    }

    {
        RxState_t state = s_rxState;
        uint8_t id = s_rxId, len = s_rxLen, idx = s_rxIdx;
        uint8_t* payload = s_rxPayload;

        switch (state) {
        case ST_MAGIC0:
            state = (byte == SERIAL_MAGIC0) ? ST_MAGIC1 : ST_MAGIC0;
            break;
        case ST_MAGIC1:
            state = (byte == SERIAL_MAGIC1) ? ST_ID :
                                              ((byte == SERIAL_MAGIC0) ? ST_MAGIC1 : ST_MAGIC0);
            break;
        case ST_ID:
            id    = byte;
            state = ST_LEN;
            break;
        case ST_LEN:
            len = byte;
            idx = 0;
            if (len > SERIAL_LINK_MAX_PAYLOAD) {
                state = ST_MAGIC0; /* bogus length, resync */
            } else {
                state = (len == 0) ? ST_CRC : ST_PAYLOAD;
            }
            break;
        case ST_PAYLOAD:
            payload[idx++] = byte;
            if (idx >= len) {
                state = ST_CRC;
            }
            break;
        case ST_CRC: {
            /* CRC is over the concatenation (id, len, payload). */
            uint8_t buf[2 + SERIAL_LINK_MAX_PAYLOAD];
            buf[0] = id;
            buf[1] = len;
            memcpy (&buf[2], payload, len);
            if (SerialCrc8 (buf, (uint32_t)len + 2U) == byte) {
                if (id < SERIAL_LINK_MAX_MSG_ID && s_handlers[id] != NULL) {
                    s_handlers[id](payload, len);
                }
            }
            state = ST_MAGIC0;
            break;
        }
        default: state = ST_MAGIC0; break;
        }

        s_rxState = state;
        s_rxId    = id;
        s_rxLen   = len;
        s_rxIdx   = idx;
    }
}

#else /* !BRD_IS_ENABLED(SERIAL_LINK) */

eSTATUS_t SerialLink_Init (void) {
    return eSTATUS_SUCCESS;
}
eSTATUS_t SerialLink_RegisterHandler (uint8_t msgId, SerialLink_Handler_t handler) {
    FJ_UNUSED (msgId);
    FJ_UNUSED (handler);
    return eSTATUS_SUCCESS;
}
eSTATUS_t SerialLink_SendFrame (uint8_t msgId, uint8_t const* pPayload, uint8_t len) {
    FJ_UNUSED (msgId);
    FJ_UNUSED (pPayload);
    FJ_UNUSED (len);
    return eSTATUS_SUCCESS;
}
void SerialLink_SendText (uint8_t const* pData, uint32_t len) {
    FJ_UNUSED (pData);
    FJ_UNUSED (len);
}
uint32_t SerialLink_GetDroppedBytes (void) {
    return 0U;
}
void SerialLink_RxUpdate (void) {
    vTaskDelay (portMAX_DELAY);
}
void SerialLink_TxUpdate (void) {
    vTaskDelay (portMAX_DELAY);
}

#endif /* BRD_IS_ENABLED(SERIAL_LINK) */
