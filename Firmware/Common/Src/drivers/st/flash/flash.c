#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "core/core.h"

#include "drivers/st/flash/W25N01GW.h"
#include "drivers/st/flash/flash.h"

FJ_DEFINE_SHARED (Flash_t, g_Flash) = { 0 };

// clang-format off
STATIC INLINE eSTATUS_t FlashWrite_ (Flash_t* pFlash, uint8_t const* pTx, uint32_t size) {
    return SpiDev_Write (&pFlash->spiDev, pTx, size);
}

STATIC INLINE eSTATUS_t FlashWriteRead (Flash_t* pFlash, uint8_t const* pTx, uint8_t* pRx, uint32_t size) {
    // clang-format on
    return SpiDev_WriteRead (&pFlash->spiDev, pTx, pRx, size);
}

STATIC uint8_t FlashReadStatusReg (Flash_t* pFlash, uint8_t regAddr) {

    uint8_t tx[]            = { W25NO1GW_INSTR_READ_SR, regAddr, 0x00U };
    uint8_t rx[sizeof (tx)] = { 0 };
    eSTATUS_t status        = FlashWriteRead (pFlash, tx, rx, sizeof (tx));

    if (STATUS_OK (status)) {
        return rx[2];
    }
    return 0xFFU;
}

// clang-format off
STATIC eSTATUS_t FlashReadJEDECID (Flash_t* pFlash, uint8_t* pManufacturerID, uint16_t* pDeviceId) {
    // clang-format on
    uint8_t tx[1U + 4U]     = { 0 };
    tx[0]                   = W25NO1GW_INSTR_JEDEC_ID;
    uint8_t rx[sizeof (tx)] = { 0 };
    eSTATUS_t status        = FlashWriteRead (pFlash, tx, rx, sizeof (tx));

    if (STATUS_OK (status)) {
        *pManufacturerID = rx[2U];
        *pDeviceId       = (uint16_t)((uint16_t)rx[3U] << 8U) | (uint16_t)rx[4U];
        return eSTATUS_SUCCESS;
    }
    return status;
}

STATIC INLINE bool FlashCheckWriteInProgress (Flash_t* pFlash) {
    return (FlashReadStatusReg (pFlash, W25NO1GW_STATUS_REG) & W25NO1GW_STATUS_WIP_BIT) > 0U;
}

// clang-format off
STATIC INLINE bool FlashWaitWriteInProgress (Flash_t* pFlash, uint32_t timeout) {
    // clang-format on
    while (FlashCheckWriteInProgress (pFlash) && timeout-- > 0U) {
        DelayMicroseconds (1U);
    }
    return timeout > 0U ? true : false;
}

STATIC INLINE bool FlashWriteEnable (Flash_t* pFlash) {

    uint8_t cmd = W25NO1GW_INSTR_WRITE_EN;
    return FlashWrite_ (pFlash, &cmd, 1U) == eSTATUS_SUCCESS;
}

STATIC eSTATUS_t FlashReset (Flash_t* pFlash) {

    bool success = FlashWaitWriteInProgress (pFlash, 1000U);
    RETURN_IF (success == false, eSTATUS_TIMEOUT, "timeout waiting for flash not busy");

    // uint8_t cmd = W25NO1GW_INSTR_DEV_RESET;
    // eSTATUS_t status = FlashWrite_ (pFlash, &cmd, 1);
    DelayMicroseconds (1000U); // Wait for 1ms after reset command

    return eSTATUS_SUCCESS;
}

// clang-format off
STATIC eSTATUS_t FlashStartProgram (Flash_t* pFlash, uint16_t columnAddr, uint8_t const* pData, uint32_t size) {
    // clang-format on

    if (size > W25NO1GW_PAGE_WRITABLE_SIZE) {
        return eSTATUS_FAILURE;
    }

    if (!FlashWaitWriteInProgress (pFlash, 1000U)) {
        return eSTATUS_TIMEOUT;
    }

    if (!FlashWriteEnable (pFlash)) {
        return eSTATUS_FAILURE;
    }
    eSTATUS_t status     = eSTATUS_SUCCESS;
    uint8_t upperColAddr = (uint8_t)(((uint32_t)columnAddr >> 8U) & 0xFFU);
    uint8_t lowerColAddr = (uint8_t)(columnAddr & 0xFFU);
    uint8_t startCmd[]   = { W25NO1GW_INSTR_PROGRAM_DATA_LOAD, upperColAddr, lowerColAddr };
    SpiDevTransaction_t transactions[] = {
        {
        .pTxData = startCmd,
        .pRxData = NULL,
        .size    = sizeof (startCmd),
        },
        {
        .pTxData = pData,
        .pRxData = NULL,
        .size    = size,
        },
    };
    SpiDev_Transactions (&pFlash->spiDev, transactions, sizeof (transactions) / sizeof (transactions[0]));
    return status;
}

STATIC eSTATUS_t FlashExecuteProgram (Flash_t* pFlash, uint16_t pageAddr) {

    uint8_t upperPageAddr = (uint8_t)(((uint32_t)pageAddr >> 8U) & 0xFFU);
    uint8_t lowerPageAddr = (uint8_t)(pageAddr & 0xFFU);
    uint8_t execCmd[]     = { W25NO1GW_INSTR_PROGRAM_EXECUTE, 0x00, upperPageAddr, lowerPageAddr };
    return FlashWrite_ (pFlash, execCmd, sizeof (execCmd));
}

static void FlashDebugSink (uint8_t const* pData, uint32_t len) {

    // TODO: decide where to start writing debug messages
    static uint32_t addr = W25NO1GW_TOTAL_PAGE_SIZE * 20U;
    addr += Flash_Write (&g_Flash, addr, pData, len);
}

eSTATUS_t Flash_Init_ (Flash_t* pOutFlash) {

    if (SpiDev_Init (&pOutFlash->spiDev) != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to initialize SPI device for flash");
        return eSTATUS_FAILURE;
    }

    eSTATUS_t status = FlashReset (pOutFlash);
    RETURN_IF (STATUS_FAIL (status), status, "Failed to reset flash devices");

    uint8_t manufacturerId = 0U;
    uint16_t flashDeviceId = 0U;
    status                 = FlashReadJEDECID (pOutFlash, &manufacturerId, &flashDeviceId);
    bool success = manufacturerId == W25NO1GW_MANUFACTURER_ID && flashDeviceId == W25NO1GW_DEVICE_ID;
    if (STATUS_FAIL (status) || success == false) {
        LOG_ERROR ("Flash Manufacturer ID: 0x%02X", manufacturerId);
        LOG_ERROR ("Flash Device ID: 0x%04X", flashDeviceId);
        return eSTATUS_FAILURE;
    }
    LoggerAddSink (FlashDebugSink);
    return status;
}

uint32_t Flash_Write (Flash_t* pFlash, uint32_t addr, uint8_t const* pData, uint32_t size) {

    if (!pData || !size) {
        return eSTATUS_FAILURE;
    }

    if (size > W25NO1GW_PAGE_WRITABLE_SIZE * W25NO1GW_PAGES_PER_BLOCK * W25NO1GW_NUM_BLOCKS) {
        return eSTATUS_FAILURE;
    }

    uint32_t bytesWritten = 0U;
    while (bytesWritten < size) {

        uint32_t currentAddr  = addr + bytesWritten;
        uint16_t pageAddr     = currentAddr / W25NO1GW_PAGE_WRITABLE_SIZE;
        uint16_t columnAddr   = currentAddr & (W25NO1GW_PAGE_WRITABLE_SIZE - 1U);
        uint32_t bytesToWrite = W25NO1GW_PAGE_WRITABLE_SIZE - columnAddr;
        bytesToWrite = bytesToWrite > (size - bytesWritten) ? (size - bytesWritten) : bytesToWrite;
        eSTATUS_t status = FlashStartProgram (pFlash, columnAddr, &pData[bytesWritten], bytesToWrite);
        RETURN_IF (STATUS_FAIL (status), status, "Failed to start flash program");

        status = FlashExecuteProgram (pFlash, pageAddr);
        RETURN_IF (STATUS_FAIL (status), status, "Failed to execute flash program");
        bytesWritten += bytesToWrite;
    }
    return bytesWritten;
}