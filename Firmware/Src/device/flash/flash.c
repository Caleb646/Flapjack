
#include "device/flash/flash.h"
#include "common.h"
#include "conf/board.h"
#include "conf/conf.h"
#include "conf/ids.h"
#include "core/core.h"
#include "core/log/logger.h"
#include "device/flash/W25N01GW.h"
#include "peripheral/bus/bus.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>


#define FLASH_VALID(pFLASH) \
    ((pFLASH) != NULL && (pFLASH)->isInitialized && BUS_VALID (&(pFLASH)->bus))

static SHARED_MEM_SECTION Flash_t g_Flash = { 0 };

FJ_STATIC FJ_INLINE eSTATUS_t FlashRead_ (vFlash_t* pFlash, uint8_t* pRx, uint32_t size) {
    return BUS_READ_BLOCK (&pFlash->bus, pRx, size);
}

// clang-format off
FJ_STATIC FJ_INLINE eSTATUS_t FlashWrite_ (vFlash_t* pFlash, uint8_t const* pTx, uint32_t size) {
    return BUS_WRITE_BLOCK (&pFlash->bus, pTx, size);
}

FJ_STATIC FJ_INLINE eSTATUS_t FlashWriteRead (vFlash_t* pFlash, uint8_t const* pTx, uint8_t* pRx, uint32_t size) {
    // clang-format on
    return BUS_WRITE_READ_BLOCK (&pFlash->bus, pTx, pRx, size);
}

FJ_STATIC uint8_t FlashReadStatusReg (vFlash_t* pFlash, uint8_t regAddr) {

    uint8_t tx[]            = { W25NO1GW_INSTR_READ_SR, regAddr, 0x00U };
    uint8_t rx[sizeof (tx)] = { 0 };
    eSTATUS_t status        = FlashWriteRead (pFlash, tx, rx, sizeof (tx));

    if (STATUS_OK (status)) {
        return rx[2];
    }
    return 0xFFU;
}

// clang-format off
FJ_STATIC eSTATUS_t FlashReadJEDECID (vFlash_t* pFlash, uint8_t* pManufacturerID, uint16_t* pDeviceId) {
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

FJ_STATIC FJ_INLINE bool FlashCheckWriteInProgress (vFlash_t* pFlash) {
    return (FlashReadStatusReg (pFlash, W25NO1GW_STATUS_REG) & W25NO1GW_STATUS_WIP_BIT) > 0U;
}

// clang-format off
FJ_STATIC FJ_INLINE eSTATUS_t FlashWaitWriteInProgress (vFlash_t* pFlash, uint32_t timeout) {
    // clang-format on
    while (FlashCheckWriteInProgress (pFlash) && timeout-- > 0U) {
        DelayMicroseconds (1U);
    }
    return timeout > 0U ? eSTATUS_SUCCESS : eSTATUS_TIMEOUT;
}

FJ_STATIC FJ_INLINE eSTATUS_t FlashWriteEnable (vFlash_t* pFlash) {

    uint8_t cmd = W25NO1GW_INSTR_WRITE_EN;
    return FlashWrite_ (pFlash, &cmd, 1U);
}

FJ_STATIC eSTATUS_t FlashReset (vFlash_t* pFlash) {

    eSTATUS_t status = FlashWaitWriteInProgress (pFlash, 1000U);
    RETURN_IF (FJ_FAIL (status), status, "timeout waiting for flash not busy");

    // uint8_t cmd = W25NO1GW_INSTR_DEV_RESET;
    // eSTATUS_t status = FlashWrite_ (pFlash, &cmd, 1);
    DelayMicroseconds (1000U); // Wait for 1ms after reset command

    return eSTATUS_SUCCESS;
}

// clang-format off
FJ_STATIC eSTATUS_t FlashStartProgram (vFlash_t* pFlash, uint16_t columnAddr, uint8_t const* pData, uint32_t size) {
    // clang-format on
    if (size > W25NO1GW_PAGE_WRITABLE_SIZE) {
        return eSTATUS_FAILURE;
    }

    eSTATUS_t status = eSTATUS_SUCCESS;

    status = FlashWaitWriteInProgress (pFlash, 1000U);
    RETURN_IF (FJ_FAIL (status), status, "timeout waiting for flash not busy");

    status = FlashWriteEnable (pFlash);
    RETURN_IF (FJ_FAIL (status), status, "Failed to enable flash write");

    uint8_t upperColAddr = (uint8_t)(((uint32_t)columnAddr >> 8U) & 0xFFU);
    uint8_t lowerColAddr = (uint8_t)(columnAddr & 0xFFU);
    uint8_t startCmd[]   = { W25NO1GW_INSTR_PROGRAM_DATA_LOAD, upperColAddr, lowerColAddr };
    BUS_CREATE_TRANSACTIONS (transactions, 2U) = { BUS_CREATE_WRITE_TRANSACTION (startCmd, sizeof (startCmd)),
                                                   BUS_CREATE_WRITE_TRANSACTION (pData, size) };
    return BUS_DO_TRANSACTIONS_BLOCK (&pFlash->bus, transactions, 2U);
}

FJ_STATIC eSTATUS_t FlashExecuteProgram (vFlash_t* pFlash, uint16_t pageAddr) {

    uint8_t upperPageAddr = (uint8_t)(((uint32_t)pageAddr >> 8U) & 0xFFU);
    uint8_t lowerPageAddr = (uint8_t)(pageAddr & 0xFFU);
    uint8_t execCmd[]     = { W25NO1GW_INSTR_PROGRAM_EXECUTE, 0x00, upperPageAddr, lowerPageAddr };
    return FlashWrite_ (pFlash, execCmd, sizeof (execCmd));
}

static void FlashDebugSink (uint8_t const* pData, uint32_t len) {

    // TODO: decide where to start writing debug messages
    static uint32_t addr = W25NO1GW_TOTAL_PAGE_SIZE * 20U;
    if (g_Flash.isInitialized == true) {
        uint32_t bytesWritten = 0U;
        FlashWrite (&g_Flash, addr, pData, len, &bytesWritten);
        addr += bytesWritten;
    }
}

eSTATUS_t FlashInit (FlashInitConf_t conf, Flash_t* pOutFlash) {

    if (FJ_IS_NULL (conf.pDevDesc) || !BUS_VALID (conf.pBus)) {
        return eSTATUS_INVALID_ARG;
    }

    eSTATUS_t status    = eSTATUS_SUCCESS;
    DevDesc_t* pDevDesc = conf.pDevDesc;
    vFlash_t* pFlash    = &g_Flash;
    if (pOutFlash != NULL) {
        pFlash = pOutFlash;
    }

    if (pFlash->isInitialized) {
        return eSTATUS_ALREADY_INITED;
    }

    memset (pFlash, 0, sizeof (Flash_t));
    pFlash->deviceId = DEV_DESC_GET_ID (pDevDesc);
    pFlash->bus      = *conf.pBus;

    status = FlashReset (pOutFlash);
    RETURN_IF (FJ_FAIL (status), status, "Failed to reset flash devices");

    uint8_t manufacturerId = 0U;
    uint16_t flashDeviceId = 0U;
    status                 = FlashReadJEDECID (pOutFlash, &manufacturerId, &flashDeviceId);
    if (FJ_FAIL (status) || !(manufacturerId == W25NO1GW_MANUFACTURER_ID && flashDeviceId == W25NO1GW_DEVICE_ID)) {
        LOG_ERROR ("Flash Manufacturer ID: 0x%02X", manufacturerId);
        LOG_ERROR ("Flash Device ID: 0x%04X", flashDeviceId);
        goto error;
    }

    pFlash->isInitialized = true;
    LoggerAddSink (FlashDebugSink);
    return status;

error:
    memset (pFlash, 0, sizeof (Flash_t));
    return status;
}

eSTATUS_t FlashStart (vFlash_t* pFlash) {

    if (!FLASH_VALID (pFlash)) {
        return eSTATUS_FAILURE;
    }

    return eSTATUS_SUCCESS;
}

eSTATUS_t FlashWrite (vFlash_t* pFlash, uint32_t addr, uint8_t const* pData, uint32_t size, uint32_t* pBytesWritten) {

    if (!FLASH_VALID (pFlash)) {
        return eSTATUS_FAILURE;
    }

    if (pData == NULL || size == 0U) {
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
        RETURN_IF (FJ_FAIL (status), status, "Failed to start flash program");

        status = FlashExecuteProgram (pFlash, pageAddr);
        RETURN_IF (FJ_FAIL (status), status, "Failed to execute flash program");
        bytesWritten += bytesToWrite;
    }

    if (pBytesWritten != NULL) {
        *pBytesWritten = bytesWritten;
    }
    return eSTATUS_SUCCESS;
}

vFlash_t const* FlashGetActiveDevice (void) {

    if (!FLASH_VALID (&g_Flash)) {
        return NULL;
    }
    return &g_Flash;
}

vFlash_t* FlashGetMutableActiveDevice (void) {

    if (!FLASH_VALID (&g_Flash)) {
        return NULL;
    }
    return &g_Flash;
}