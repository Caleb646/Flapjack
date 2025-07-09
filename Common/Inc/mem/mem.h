
#ifndef MEM_H
#define MEM_H

#include "common.h"
#include "mem/ring_buff.h"
#include "stdint.h"

/**
 * \brief Ring Buffer forward declaration
 */
typedef struct RingBuff_ RingBuff;

#define MEM_U32_ALIGN4(addr)         ((uint32_t)(addr) & ((uint32_t)~0x3U))

// SRAM4 = 0x38000000 - 0x3800FFFF
#define MEM_SRAM_4_START             0x38000000U
#define MEM_SRAM_4_END               0x3800FFFFU

/*
 * Mailbox for each core
 */
#define MEM_SHARED_MAILBOX_LEN       8U
#define MEM_SHARED_MAILBOX_CM7_START MEM_U32_ALIGN4 (MEM_SRAM_4_START)
#define MEM_SHARED_MAILBOX_CM4_START \
    MEM_U32_ALIGN4 (MEM_SHARED_MAILBOX_CM7_START + MEM_SHARED_MAILBOX_LEN)
#define MEM_SHARED_MAILBOX_END \
    (MEM_SHARED_MAILBOX_CM4_START + MEM_SHARED_MAILBOX_LEN)

/*
 * Shared Ring Buffer for CM4 to write to the UART
 */
#define MEM_SHARED_CM4_UART_RINGBUFF_START \
    MEM_U32_ALIGN4 (MEM_SHARED_MAILBOX_END)
#define MEM_SHARED_CM4_UART_RINGBUFF_TOTAL_LEN (sizeof (RingBuff) + 256U)
#define MEM_SHARED_CM4_UART_RINGBUFF_END \
    (MEM_SHARED_CM4_UART_RINGBUFF_START + MEM_SHARED_CM4_UART_RINGBUFF_TOTAL_LEN)
/*
 * Shared Ring Buffer for CM7 to write to the UART
 */
#define MEM_SHARED_CM7_UART_RINGBUFF_START \
    MEM_U32_ALIGN4 (MEM_SHARED_CM4_UART_RINGBUFF_END)
#define MEM_SHARED_CM7_UART_RINGBUFF_TOTAL_LEN (sizeof (RingBuff) + 8192U)
#define MEM_SHARED_CM7_UART_RINGBUFF_END \
    (MEM_SHARED_CM7_UART_RINGBUFF_START + MEM_SHARED_CM7_UART_RINGBUFF_TOTAL_LEN)

STATIC_ASSERT (MEM_SHARED_CM7_UART_RINGBUFF_END <= MEM_SRAM_4_END, "");

#endif // MEM_H
