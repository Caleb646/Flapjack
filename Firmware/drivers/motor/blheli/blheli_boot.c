/*
 * Client for the BLHeli SiLabs bootloader, bit-banged on a motor signal pin.
 *
 * The wire is one half-duplex line at 19200 8N1, idle high: the FC drives it
 * to send and releases it to a pull-up to listen. Command framing, acks and
 * timeouts follow Betaflight's serial_4way_avrootloader.c, which is what every
 * BLHeli_S passthrough in the field actually exercises; the bootloader side is
 * documented by BLHeliBoot.asm / AVRootloader.inc in BLHeliSuite.
 */

#include "drivers/motor/blheli/blheli.h"

#include "hal.h"

#include "core/core.h"

#include "drivers/io/gpio.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

/* 19200 baud, the only rate the SiLabs BLHeli bootloader accepts. The 0.08 us
 * truncation is 0.8 us over a 10-bit frame, against a 26 us sampling margin. */
#define BIT_US               52U
#define START_BIT_TIMEOUT_US 2000U

#define ACK_SUCCESS          0x30U

#define CMD_RUN              0x00U
#define CMD_PROG_FLASH       0x01U
#define CMD_ERASE_FLASH      0x02U
#define CMD_READ_FLASH       0x03U
#define CMD_SET_BUFFER       0xFEU
#define CMD_SET_ADDRESS      0xFFU

#define CMD_TIMEOUT_MS       10U
#define BUFFER_TIMEOUT_MS    100U
#define PROG_TIMEOUT_MS      500U
#define ERASE_TIMEOUT_MS     3000U
#define RUN_SETTLE_US        2000U

/*
 * A running BLHeli_S jumps to its bootloader once it sees the signal line held
 * high for ~15 ms with no throttle signal, and an ESC that is still playing its
 * power-up tune gets there later. Each attempt is ~11 ms of traffic followed by
 * idle-high, so the period must leave well over 15 ms of steady high or the
 * attempts themselves keep the ESC out of its bootloader. For the same reason
 * the first attempt waits out ENTRY_IDLE_US: sent any sooner, it is the low
 * edge that restarts the ESC's count.
 *
 * The window is a bench estimate, not a spec figure. It is also the boot-time
 * cost when the ESCs are unpowered (USB-only bench work).
 */
#define ENTRY_IDLE_US        20000U
#define ENTRY_WINDOW_US      2000000U
#define ENTRY_RETRY_US       100000U
#define ENTRY_MIN_ATTEMPTS   3U

typedef struct {
    GPIO_TypeDef* pPort;
    uint32_t pin;
    uint32_t moderShift;
} Line_t;

static Line_t Line_Make (GPIO_TypeDef* pPort, uint32_t pin) {

    Line_t line = { .pPort = pPort, .pin = pin, .moderShift = 2U * (uint32_t)__builtin_ctz (pin) };
    return line;
}

static void Line_Output (Line_t const* p) {

    p->pPort->MODER = (p->pPort->MODER & ~(3UL << p->moderShift)) | (1UL << p->moderShift);
}

/* The pull-up (set once in Line_Init) is what holds the line idle-high. */
static void Line_Input (Line_t const* p) {

    p->pPort->MODER &= ~(3UL << p->moderShift);
}

static void Line_Set (Line_t const* p, bool high) {

    if (high) {
        GPIO_SetHigh (p->pPort, (uint16_t)p->pin);
    } else {
        GPIO_SetLow (p->pPort, (uint16_t)p->pin);
    }
}

static bool Line_IsHigh (Line_t const* p) {

    return (p->pPort->IDR & p->pin) != 0U;
}

static void Line_Init (Line_t const* p) {

    GPIO_ENABLE_CLOCK (p->pPort);
    Line_Set (p, true);
    GPIO_InitTypeDef init = { 0 };
    init.Pin              = p->pin;
    init.Mode             = GPIO_MODE_OUTPUT_PP;
    init.Pull             = GPIO_PULLUP;
    init.Speed            = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init (p->pPort, &init);
    Line_Input (p);
}

static void WaitUntil (uint32_t usTarget) {

    while ((int32_t)(GetMicroseconds () - usTarget) < 0) {
    }
}

/*
 * Interrupts are masked per byte, not per transfer: an ISR between bytes only
 * stretches the idle gap, and the receiver times every bit from the start edge
 * it saw, so a late detection costs margin within one byte instead of
 * accumulating.
 */
static void PutByte (Line_t const* p, uint8_t b) {

    /* A leading idle bit doubles as the previous byte's stop bit, so the stop
     * bit itself need not be waited out. */
    uint32_t frame         = 0x01U | ((uint32_t)b << 2U) | (1UL << 10U);
    uint32_t const primask = __get_PRIMASK ();
    __disable_irq ();

    uint32_t t = GetMicroseconds ();
    for (;;) {
        Line_Set (p, (frame & 0x01U) != 0U);
        frame >>= 1U;
        if (frame == 0U) {
            break;
        }
        t += BIT_US;
        WaitUntil (t);
    }
    __set_PRIMASK (primask);
}

static bool GetByte (Line_t const* p, uint8_t* pOut) {

    uint32_t const primask = __get_PRIMASK ();
    __disable_irq ();

    bool ok              = true;
    uint32_t const start = GetMicroseconds ();
    while (Line_IsHigh (p)) {
        if ((GetMicroseconds () - start) >= START_BIT_TIMEOUT_US) {
            ok = false;
            break;
        }
    }

    uint32_t bits = 0U;
    if (ok) {
        uint32_t t = GetMicroseconds () + (BIT_US / 2U);
        for (uint32_t i = 0; i < 10U; ++i) {
            WaitUntil (t);
            if (Line_IsHigh (p)) {
                bits |= 1UL << i;
            }
            t += BIT_US;
        }
        ok = ((bits & 0x01U) == 0U) && ((bits & (1UL << 9U)) != 0U);
    }
    __set_PRIMASK (primask);

    *pOut = (uint8_t)(bits >> 1U);
    return ok;
}

/* CRC bytes are little-endian on the wire. Nothing is CRC-framed until the
 * bootloader has answered the handshake. */
static void SendBuf (Line_t const* p, uint8_t const* pData, uint32_t len, bool withCrc) {

    Line_Output (p);
    for (uint32_t i = 0; i < len; ++i) {
        PutByte (p, pData[i]);
    }
    if (withCrc) {
        uint16_t const crc = BLHeli_Crc16 (0U, pData, len);
        PutByte (p, (uint8_t)crc);
        PutByte (p, (uint8_t)(crc >> 8U));
    }
    Line_Input (p);
}

static bool ReadBuf (Line_t const* p, uint8_t* pOut, uint32_t len, bool withCrc) {

    for (uint32_t i = 0; i < len; ++i) {
        if (!GetByte (p, &pOut[i])) {
            return false;
        }
    }
    uint8_t crc[2] = { 0 };
    if (withCrc && (!GetByte (p, &crc[0]) || !GetByte (p, &crc[1]))) {
        return false;
    }
    uint8_t ack = 0U;
    if (!GetByte (p, &ack)) {
        return false;
    }
    if (withCrc && BLHeli_Crc16 (0U, pOut, len) != (uint16_t)(crc[0] | ((uint16_t)crc[1] << 8U))) {
        return false;
    }
    return ack == ACK_SUCCESS;
}

static bool GetAck (Line_t const* p, uint32_t msTimeout, uint8_t* pAck) {

    uint32_t const start = GetMicroseconds ();
    do {
        if (GetByte (p, pAck)) {
            return true;
        }
    } while ((GetMicroseconds () - start) < (msTimeout * 1000U));
    return false;
}

static bool SendCmd (Line_t const* p, uint8_t const* pCmd, uint32_t len, uint32_t msTimeout) {

    SendBuf (p, pCmd, len, true);
    uint8_t ack = 0U;
    return GetAck (p, msTimeout, &ack) && ack == ACK_SUCCESS;
}

static bool SetAddress (Line_t const* p, uint16_t addr) {

    uint8_t const cmd[] = { CMD_SET_ADDRESS, 0U, (uint8_t)(addr >> 8U), (uint8_t)addr };
    return SendCmd (p, cmd, sizeof (cmd), CMD_TIMEOUT_MS);
}

static bool Connect (Line_t const* p, uint16_t* pOutSignature) {

    /* The zeros and 0x0D give the bootloader edges to find the byte framing on
     * before the "BLHeli" sign. F4 7D is BLHeli_Crc16 of "BLHeli" alone. */
    static uint8_t const bootInit[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0x0D, 'B', 'L', 'H', 'e', 'l', 'i', 0xF4, 0x7D };
    SendBuf (p, bootInit, sizeof (bootInit), false);

    /* "471x" boot message, signature high/low, boot version, boot pages. */
    uint8_t info[8] = { 0 };
    if (!ReadBuf (p, info, sizeof (info), false) || memcmp (info, "471", 3U) != 0) {
        return false;
    }
    *pOutSignature = (uint16_t)(((uint16_t)info[4] << 8U) | info[5]);
    return true;
}

static bool ReadSettings (Line_t const* p, uint8_t settings[BLHELI_SETTINGS_SIZE]) {

    if (!SetAddress (p, BLHELI_SETTINGS_ADDR)) {
        return false;
    }
    uint8_t const cmd[] = { CMD_READ_FLASH, BLHELI_SETTINGS_SIZE };
    SendBuf (p, cmd, sizeof (cmd), true);
    return ReadBuf (p, settings, BLHELI_SETTINGS_SIZE, true);
}

static bool WriteSettings (Line_t const* p, uint8_t const settings[BLHELI_SETTINGS_SIZE]) {

    /* SiLabs flash only programs erased bytes, so the page goes first. The
     * bootloader refuses anything at or above itself (0x1C00). */
    uint8_t const erase[] = { CMD_ERASE_FLASH, 0x01U };
    if (!SetAddress (p, BLHELI_SETTINGS_ADDR) || !SendCmd (p, erase, sizeof (erase), ERASE_TIMEOUT_MS)) {
        return false;
    }

    if (!SetAddress (p, BLHELI_SETTINGS_ADDR)) {
        return false;
    }
    /* The bootloader acks SET_BUFFER only once its data has arrived, so a reply
     * to the header alone means it rejected it. */
    uint8_t const setBuffer[] = { CMD_SET_BUFFER, 0U, 0U, BLHELI_SETTINGS_SIZE };
    SendBuf (p, setBuffer, sizeof (setBuffer), true);
    uint8_t ack = 0U;
    if (GetAck (p, CMD_TIMEOUT_MS, &ack)) {
        return false;
    }
    if (!SendCmd (p, settings, BLHELI_SETTINGS_SIZE, BUFFER_TIMEOUT_MS)) {
        return false;
    }

    uint8_t const prog[] = { CMD_PROG_FLASH, 0x01U };
    return SendCmd (p, prog, sizeof (prog), PROG_TIMEOUT_MS);
}

static eSTATUS_t Configure (Line_t const* p, BLHeliSettings_t const* pWanted, uint32_t escNum) {

    uint8_t settings[BLHELI_SETTINGS_SIZE] = { 0 };
    if (!ReadSettings (p, settings)) {
        LOG_WARN ("ESC %u: settings read failed", (unsigned)escNum);
        return eSTATUS_FAILURE;
    }

    bool changed = false;
    if (STATUS_FAIL (BLHeli_MergeSettings (settings, pWanted, &changed))) {
        LOG_WARN ("ESC %u: layout %u is not BLHeli_S, left unchanged", (unsigned)escNum, (unsigned)settings[BLHELI_OFS_LAYOUT_REVISION]);
        return eSTATUS_UNSUPPORTED;
    }
    if (!changed) {
        LOG_INFO ("ESC %u: settings already match", (unsigned)escNum);
        return eSTATUS_SUCCESS;
    }

    uint8_t readback[BLHELI_SETTINGS_SIZE] = { 0 };
    if (!WriteSettings (p, settings) || !ReadSettings (p, readback) || memcmp (settings, readback, sizeof (settings)) != 0) {
        LOG_ERROR ("ESC %u: settings write failed", (unsigned)escNum);
        return eSTATUS_FAILURE;
    }
    LOG_INFO ("ESC %u: settings written", (unsigned)escNum);
    return eSTATUS_SUCCESS;
}

static eSTATUS_t ApplyOne (Line_t const* p, BLHeliSettings_t const* pWanted, uint32_t escNum, uint32_t usDeadline) {

    uint16_t signature = 0U;
    uint32_t attempts  = 0U;
    for (;;) {
        uint32_t const usAttempt = GetMicroseconds ();
        if (Connect (p, &signature)) {
            break;
        }
        ++attempts;
        if (attempts >= ENTRY_MIN_ATTEMPTS && (int32_t)(GetMicroseconds () - usDeadline) >= 0) {
            LOG_WARN ("ESC %u: no bootloader response (ESC unpowered?)", (unsigned)escNum);
            return eSTATUS_TIMEOUT;
        }
        WaitUntil (usAttempt + ENTRY_RETRY_US);
    }

    LOG_INFO ("ESC %u: bootloader signature %x", (unsigned)escNum, (unsigned)signature);
    eSTATUS_t const status = Configure (p, pWanted, escNum);

    /* PutByte returns as the last stop bit starts, and the caller pulls the
     * line low next - which would turn that stop bit into a framing error.
     * The pull-up carries it while the bootloader takes the command. */
    uint8_t const run[] = { CMD_RUN, 0U };
    SendBuf (p, run, sizeof (run), true);
    WaitUntil (GetMicroseconds () + RUN_SETTLE_US);
    return status;
}

eSTATUS_t BLHeli_ApplySettings (GPIO_TypeDef* const pPorts[], uint32_t const pins[], BLHeliSettings_t const pWanted[], uint32_t count) {

    /* Raise every line first so all ESCs head for their bootloaders together,
     * rather than each one only starting once its predecessor is done. */
    for (uint32_t i = 0; i < count; ++i) {
        Line_t const line = Line_Make (pPorts[i], pins[i]);
        Line_Init (&line);
    }
    WaitUntil (GetMicroseconds () + ENTRY_IDLE_US);

    eSTATUS_t status          = eSTATUS_SUCCESS;
    uint32_t const usDeadline = GetMicroseconds () + ENTRY_WINDOW_US;
    for (uint32_t i = 0; i < count; ++i) {
        Line_t const line = Line_Make (pPorts[i], pins[i]);
        if (STATUS_FAIL (ApplyOne (&line, &pWanted[i], i + 1U, usDeadline))) {
            status = eSTATUS_FAILURE;
        }
        /* Low, not idle-high: a steady high would send the ESC straight back
         * into its bootloader instead of waiting for DShot. */
        Line_Set (&line, false);
        Line_Output (&line);
    }
    return status;
}
