#ifndef DRIVERS_SERIAL_SERIAL_LINK_H
#define DRIVERS_SERIAL_SERIAL_LINK_H

/*
 * Sole owner of the SERIAL_LINK UART. Every byte in or out of that peripheral
 * passes through here - this is the only module that calls UartPort_Init or
 * UartPort_Write on it.
 *
 * Three consumers share the wire:
 *
 *   logger  -> SerialLink_SendText()   TX, raw ASCII
 *   simlink -> SerialLink_SendFrame()  TX+RX, ids 1-5
 *   shell   -> RegisterHandler()       RX, id 6
 *
 * Two streams, one wire:
 *
 *   <{"type":"debug",...}>\r\n     log text, passes through untouched
 *   [0xAA][0x55][id][len][pb][crc] framed binary
 *
 * They are unambiguous because log text is 7-bit ASCII and the frame magic is
 * 0xAA, so a text byte can never open a frame. That is load-bearing: NEVER put
 * a non-ASCII byte through SendText. The payoff is that a plain terminal still
 * shows readable logs and the host needs no unwrapping for them.
 *
 * TX is split by what can afford to wait. Frames are written synchronously by
 * their producer, so actuator timing is unchanged by this module. Only log
 * text is buffered, drained in bounded chunks by the TX task; a mutex keeps
 * the two from interleaving mid-frame. So a log flood can never corrupt or
 * meaningfully delay an actuator command - it loses log lines instead,
 * counted by SerialLink_GetDroppedBytes().
 */

#include "core/core.h"

#include <stdbool.h>
#include <stdint.h>

/*
 * Frame message id registry - one namespace shared by every client.
 * 1-5 belong to the sim link (SIM_MSG_* in drivers/sim_link/sim_link.h).
 */
#define SERIAL_MSG_SHELL_CMD     6U
#define SERIAL_LINK_MAX_MSG_ID   8U

/* Comfortably above every message on the wire: SensorData_size (45) is the
 * largest sim message and the shell's Command_size is 11. It was sized to the
 * old RcInput_size (96), which retired with the sim-link RC path; the headroom
 * is kept rather than trimmed, since three buffers depend on it and nothing
 * gains from shaving them. */
#define SERIAL_LINK_MAX_PAYLOAD  96U
#define SERIAL_LINK_MAX_FRAME    (5U + SERIAL_LINK_MAX_PAYLOAD)

typedef void (*SerialLink_Handler_t) (uint8_t const* pPayload, uint8_t len);

// One-time init: claims the UART, creates the rings/stream, registers the
// logger sink. Call once from main.c before the scheduler starts.
eSTATUS_t SerialLink_Init (void);

// Blocking pumps. Each returns after one unit of work, so the caller is a bare
// for(;;) loop - see tasks/serial_link/, where the task bodies live.
void SerialLink_RxUpdate (void);
void SerialLink_TxUpdate (void);

// Route inbound frames carrying msgId to handler. Called from RxTask context.
eSTATUS_t SerialLink_RegisterHandler (uint8_t msgId, SerialLink_Handler_t handler);

// Queue a framed message. All-or-nothing: a frame is never partially written,
// so FAILURE means it was dropped whole rather than corrupting the stream.
eSTATUS_t SerialLink_SendFrame (uint8_t msgId, uint8_t const* pPayload, uint8_t len);

// Queue raw ASCII (log text). Best-effort - excess is dropped and counted.
// Task context only, matching the logger's existing contract.
void SerialLink_SendText (uint8_t const* pData, uint32_t len);

// Log bytes dropped for want of TX space. Non-zero means the link is saturated;
// lower CFG_LOG_LEVEL or raise the baud.
uint32_t SerialLink_GetDroppedBytes (void);

#endif // DRIVERS_SERIAL_SERIAL_LINK_H
