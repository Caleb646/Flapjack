#ifndef DEVICE_FLASH_W25N01GW_H
#define DEVICE_FLASH_W25N01GW_H

#define W25NO1GW_MANUFACTURER_ID         0xEFU
#define W25NO1GW_DEVICE_ID               0xBA21U

#define W25NO1GW_TOTAL_PAGE_SIZE         (2048U + 64U) // 2KB + 64B spare (for ECC)
#define W25NO1GW_PAGE_WRITABLE_SIZE      2048U
#define W25NO1GW_PAGES_PER_BLOCK         64U
#define W25NO1GW_NUM_BLOCKS              1024U

/*
 * Register Addresses
 */
#define W25NO1GW_PROTECTION_REG          0xA0U
#define W25NO1GW_CONFIG_REG              0xB0U
#define W25NO1GW_STATUS_REG              0xC0U

/*
 * Instructions
 */
// (SENT) 0Fh or 05h --> (SENT) SR Addr --> (RECVD) S7-0 --> (RECVD) S7-0 --> (RECVD) S7-0 --> (RECVD) S7-0 --> (RECVD) S7-0 --> (RECVD) S7-0 --> (RECVD) S7-0
#define W25NO1GW_INSTR_READ_SR           0x05U
// (SENT) 02h --> (SENT) 1Fh or 01h --> (SENT) SR Addr --> (SENT) S7-0
#define W25NO1GW_INSTR_WRITE_SR          0x01U
#define W25NO1GW_INSTR_WRITE_EN          0x06U
#define W25NO1GW_INSTR_WRITE_DI          0x04U
#define W25NO1GW_INSTR_BLOCK_ERASE       0xD8U
#define W25NO1GW_INSTR_DEV_RESET         0xFFU
#define W25NO1GW_INSTR_JEDEC_ID          0x9FU
// (SENT) CA15-8 --> (SENT) CA7-0 --> (SENT) Data-0 --> (SENT) Data-1 --> (SENT) Data-2 --> (SENT) Data-N
#define W25NO1GW_INSTR_PROGRAM_DATA_LOAD 0x02U
// 10h --> Dummy --> PA15-8 --> PA7-0
#define W25NO1GW_INSTR_PROGRAM_EXECUTE   0x10U

/*
 * Bits
 */
// Write in progress bit (if set, device is busy in write/erase operation)
#define W25NO1GW_STATUS_WIP_BIT          (1U << 0u)


#endif // DEVICE_FLASH_W25N01GW_H