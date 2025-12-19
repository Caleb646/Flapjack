#include "peripheral/bus/spi.h"
#include "common.h"
#include "conf/board.h"
#include "conf/conf.h"
#include "core/core.h"
#include "core/log/logger.h"
#include "hal.h"
#include "peripheral/bus/common.h"
#include "peripheral/gpio.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define SPI_VALID(pBUS) ((pBUS) != NULL && (pBUS)->isInitialized == true)

static SHARED_MEM_SECTION SPIBus_t g_SPIBusses[SPI_MAX_BUSES] = { 0 };

static SPI_TypeDef* SPIGetInstanceById (eBUS_ID_t busId) {

    switch (busId) {
    case eSPI_1_BUS_ID: return SPI1;
    case eSPI_2_BUS_ID: return SPI2;
    case eSPI_3_BUS_ID: return SPI3;
    case eSPI_4_BUS_ID: return SPI4;
    case eSPI_5_BUS_ID: return SPI5;
    default: LOG_ERROR ("Invalid SPI bus ID"); return NULL;
    }
}

static FJ_INLINE vIO_t* SPIGetGPIOByDeviceId (vSPIBus_t* pBus, eDEVICE_ID_t deviceId) {

    for (uint32_t i = 0; i < pBus->nDevices; ++i) {

        if (pBus->connectedDevices.ids[i] == deviceId) {
            return pBus->connectedDevices.gpios[i];
        }
    }
    return NULL;
}

static FJ_INLINE bool SPIBeginOperation (vSPIBus_t* pBus, eDEVICE_ID_t deviceId) {

    if (SPI_VALID (pBus) == false) {
        LOG_ERROR ("Failed to get SPI bus by ID");
        return false;
    }

    if (pBus->activeOperation.pNss != NULL) {
        LOG_ERROR ("SPI bus is already in a transaction");
        return false;
    }

    vIO_t* pGPIO = SPIGetGPIOByDeviceId (pBus, deviceId);
    if (pGPIO == NULL) {
        LOG_ERROR ("Failed to get GPIO for SPI device ID");
        return false;
    }
    pBus->activeOperation.pNss     = pGPIO;
    pBus->activeOperation.deviceId = deviceId;
    HAL_GPIO_WritePin (pGPIO->pPort, pGPIO->pin, GPIO_PIN_RESET);
    return true;
}

static FJ_INLINE void SPIEndOperation (vSPIBus_t* pBus) {

    if (FJ_IS_NULL (pBus->activeOperation.pNss)) {
        LOG_ERROR ("SPI bus is not in a transaction");
        return;
    }

    HAL_GPIO_WritePin (pBus->activeOperation.pNss->pPort, pBus->activeOperation.pNss->pin, GPIO_PIN_SET);
    pBus->activeOperation.pNss     = NULL;
    pBus->activeOperation.deviceId = 0;
}

#define SPI_INIT_CLOCK(pSTATUS, BUS_ID, RCC_PERIPH_CLK_SELECTION, RCC_SPI_CLK_SELECTION) \
    do {                                                                                 \
        RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = { 0 };                            \
        PeriphClkInitStruct.PeriphClockSelection     = (RCC_PERIPH_CLK_SELECTION);       \
        PeriphClkInitStruct.Spi123ClockSelection     = (RCC_SPI_CLK_SELECTION);          \
        if ((BUS_ID) > eSPI_3_BUS_ID) {                                                  \
            PeriphClkInitStruct.Spi123ClockSelection = 0;                                \
            PeriphClkInitStruct.Spi45ClockSelection  = (RCC_SPI_CLK_SELECTION);          \
        }                                                                                \
        if (HAL_RCCEx_PeriphCLKConfig (&PeriphClkInitStruct) != HAL_OK) {                \
            LOG_ERROR ("Failed to configure SPI [%u] clock", (BUS_ID));                  \
            *(pSTATUS) = eSTATUS_FAIL;                                                   \
        }                                                                                \
        *(pSTATUS) = eSTATUS_OK;                                                         \
    } while (0)

static eSTATUS_t SPIClockInit (vSPIBus_t* pBus, SPIInitConf_t conf) {

    FJ_UNUSED (conf);
    eBUS_ID_t busId  = pBus->busId;
    eSTATUS_t status = eSTATUS_OK;
    if (busId == eSPI_1_BUS_ID) {
        __HAL_RCC_SPI1_CLK_ENABLE ();
        SPI_INIT_CLOCK (&status, eSPI_1_BUS_ID, RCC_PERIPHCLK_SPI1, RCC_SPI123CLKSOURCE_PLL);
        return status;
    }
    if (busId == eSPI_2_BUS_ID) {
        __HAL_RCC_SPI2_CLK_ENABLE ();
        SPI_INIT_CLOCK (&status, eSPI_2_BUS_ID, RCC_PERIPHCLK_SPI2, RCC_SPI123CLKSOURCE_PLL);
        return status;
    }
    if (busId == eSPI_3_BUS_ID) {
        __HAL_RCC_SPI3_CLK_ENABLE ();
        SPI_INIT_CLOCK (&status, eSPI_3_BUS_ID, RCC_PERIPHCLK_SPI3, RCC_SPI123CLKSOURCE_PLL);
        return status;
    }
    if (busId == eSPI_4_BUS_ID) {
        __HAL_RCC_SPI4_CLK_ENABLE ();
        SPI_INIT_CLOCK (&status, eSPI_4_BUS_ID, RCC_PERIPHCLK_SPI4, RCC_SPI45CLKSOURCE_PCLK2);
        return status;
    }
    if (busId == eSPI_5_BUS_ID) {
        __HAL_RCC_SPI5_CLK_ENABLE ();
        SPI_INIT_CLOCK (&status, eSPI_5_BUS_ID, RCC_PERIPHCLK_SPI5, RCC_SPI45CLKSOURCE_PCLK2);
        return status;
    }
    LOG_ERROR ("Unsupported SPI bus ID");
    return eSTATUS_FAIL;
}

static eSTATUS_t SPIInitGPIO (vSPIBus_t* pBus, SPIInitConf_t conf) {

    eSTATUS_t status          = eSTATUS_OK;
    GPIODesc_t* pSck          = BUS_DESC_SPI_GET_SCK (conf.pBusDesc);
    GPIODesc_t* pMiso         = BUS_DESC_SPI_GET_MISO (conf.pBusDesc);
    GPIODesc_t* pMosi         = BUS_DESC_SPI_GET_MOSI (conf.pBusDesc);
    SPIDeviceDesc_t* pDevices = BUS_DESC_SPI_GET_CONNECTED_DEVS (conf.pBusDesc);
    uint32_t nDevices         = BUS_DESC_SPI_GET_NUM_CONNECTED_DEVS (conf.pBusDesc);

    RETURN_IF_NULL (pSck, eSTATUS_NULL_ARG, "SPIInitGPIO: NULL SCK pin configuration");
    RETURN_IF_NULL (pMiso, eSTATUS_NULL_ARG, "SPIInitGPIO: NULL MISO pin configuration");
    RETURN_IF_NULL (pMosi, eSTATUS_NULL_ARG, "SPIInitGPIO: NULL MOSI pin configuration");
    RETURN_IF_NULL (pDevices, eSTATUS_NULL_ARG, "SPIInitGPIO: NULL connected devices pointer");
    RETURN_IF (nDevices == 0, eSTATUS_INVALID_ARG, "SPIInitGPIO: No connected devices configured for SPI bus");

    status = GPIO_INIT (pBus->busId, pSck);
    RETURN_IF (FJ_FAIL (status), status, "Failed to initialize SPI SCK GPIO");

    status = GPIO_INIT (pBus->busId, pMiso);
    RETURN_IF (FJ_FAIL (status), status, "Failed to initialize SPI MISO GPIO");

    status = GPIO_INIT (pBus->busId, pMosi);
    RETURN_IF (FJ_FAIL (status), status, "Failed to initialize SPI MOSI GPIO");

    for (uint32_t i = 0; i < nDevices; ++i) {

        eDEVICE_ID_t deviceId = SPI_DEV_DESC_GET_ID (&pDevices[i]);
        GPIODesc_t* pNss      = SPI_DEV_DESC_GET_NSS (&pDevices[i]);
        RETURN_IF_NULL (pNss, eSTATUS_NULL_ARG, "SPIInitGPIO: NULL NSS pin configuration for device");

        status = GPIO_INIT (pBus->busId, pNss);
        RETURN_IF (FJ_FAIL (status), status, "Failed to initialize SPI NSS GPIO");

        pBus->connectedDevices.gpios[i] = GPIOGetIOfromId (pNss->id);
        pBus->connectedDevices.ids[i]   = deviceId;
    }
    pBus->nDevices = nDevices;
    return status;
}

vSPIBus_t* SPIGetBusById (eBUS_ID_t busId) {

    uint32_t busIndex = SPI_BUS_ID_TO_IDX (busId);
    if (!BUS_ID_IS_SPI (busId) || busIndex >= SPI_MAX_BUSES) {
        LOG_ERROR ("Invalid SPI bus ID");
        return NULL;
    }
    return &g_SPIBusses[busIndex];
}

eSTATUS_t SPIInit (SPIInitConf_t conf, vSPIBus_t* pOutBus) {

    if (FJ_IS_NULL (conf.pBusDesc) || !BUS_DESC_IS_SPI (conf.pBusDesc)) {
        LOG_ERROR ("SPIInit: NULL or invalid BusDesc_t pointer");
        return eSTATUS_INVALID_ARG;
    }

    if (FJ_IS_NULL (BUS_DESC_SPI_GET_CONNECTED_DEVS (conf.pBusDesc))) {
        LOG_ERROR ("SPIInit: NULL connected devices pointer in BusDesc_t");
        return eSTATUS_INVALID_ARG;
    }

    if (BUS_DESC_SPI_GET_NUM_CONNECTED_DEVS (conf.pBusDesc) == 0) {
        LOG_ERROR ("SPIInit: No connected devices configured for SPI bus");
        return eSTATUS_INVALID_ARG;
    }

    eSTATUS_t status    = eSTATUS_OK;
    BusDesc_t* pBusDesc = conf.pBusDesc;
    eBUS_ID_t busId     = BUS_DESC_GET_ID (pBusDesc);
    vSPIBus_t* pBus     = SPIGetBusById (busId);

    if (pOutBus != NULL) {
        pBus = pOutBus;
    }
    RETURN_IF_NULL (pBus, eSTATUS_FAIL, "Failed to get SPI bus by ID");
    /* Let the first device to initialize the spi bus setup all of the given nss GPIO pins. */
    if (pBus->isInitialized) {
        LOG_WARN ("SPI bus %u is already initialized", busId);
        return eSTATUS_ALREADY_INITED;
    }

    LOG_INFO ("Initializing SPI bus [%u]", busId);
    memset (pBus, 0, sizeof (vSPIBus_t));
    pBus->busId                   = busId;
    pBus->handle.Instance         = SPIGetInstanceById (busId);
    pBus->handle.Init.Mode        = SPI_MODE_MASTER;
    pBus->handle.Init.Direction   = SPI_DIRECTION_2LINES;
    pBus->handle.Init.DataSize    = SPI_DATASIZE_8BIT;
    pBus->handle.Init.CLKPolarity = SPI_POLARITY_LOW;
    pBus->handle.Init.CLKPhase    = SPI_PHASE_1EDGE;
    // Manage NSS manually
    pBus->handle.Init.NSS = SPI_NSS_SOFT;
    // TODO: set based on pConf->speed
    pBus->handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
    pBus->handle.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    pBus->handle.Init.TIMode            = SPI_TIMODE_DISABLE;
    pBus->handle.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    pBus->handle.Init.CRCPolynomial     = 0x0;
    // Manage NSS manually
    pBus->handle.Init.NSSPMode                   = SPI_NSS_PULSE_DISABLE;
    pBus->handle.Init.NSSPolarity                = SPI_NSS_POLARITY_LOW;
    pBus->handle.Init.FifoThreshold              = SPI_FIFO_THRESHOLD_01DATA;
    pBus->handle.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
    pBus->handle.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
    pBus->handle.Init.MasterSSIdleness           = SPI_MASTER_SS_IDLENESS_00CYCLE;
    pBus->handle.Init.MasterInterDataIdleness    = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
    pBus->handle.Init.MasterReceiverAutoSusp     = SPI_MASTER_RX_AUTOSUSP_DISABLE;
    pBus->handle.Init.MasterKeepIOState          = SPI_MASTER_KEEP_IO_STATE_DISABLE;
    pBus->handle.Init.IOSwap                     = SPI_IO_SWAP_DISABLE;

    status = SPIClockInit (pBus, conf);
    GOTO_IF (FJ_FAIL (status), error, "Failed to initialize SPI clock");

    status = HAL_SPI_Init (&pBus->handle);
    GOTO_IF (FJ_FAIL (status), error, "Failed to initialize HAL for SPI bus");

    status = SPIInitGPIO (pBus, conf);
    GOTO_IF (FJ_FAIL (status), error, "Failed to initialize SPI GPIO");

    pBus->isInitialized = true;
    return eSTATUS_OK;

error:
    memset (pBus, 0, sizeof (vSPIBus_t));
    return eSTATUS_FAIL;
}

eSTATUS_t SPIRead_Blocking (vSPIBus_t* pBus, eDEVICE_ID_t deviceId, uint8_t* pData, size_t size) {

    eSTATUS_t status = eSTATUS_OK;
    if (SPIBeginOperation (pBus, deviceId) == false) {
        return eSTATUS_FAIL;
    }

    if (HAL_SPI_Receive (&pBus->handle, pData, size, HAL_MAX_DELAY) != HAL_OK) {
        LOG_ERROR ("Failed to read data from SPI bus");
        status = eSTATUS_FAIL;
    }

    SPIEndOperation (pBus);
    return status;
}

eSTATUS_t SPIWrite_Blocking (vSPIBus_t* pBus, eDEVICE_ID_t deviceId, uint8_t const* pData, size_t size) {

    eSTATUS_t status = eSTATUS_OK;
    if (!SPIBeginOperation (pBus, deviceId)) {
        return eSTATUS_FAIL;
    }

    if (HAL_SPI_Transmit (&pBus->handle, pData, size, HAL_MAX_DELAY) != HAL_OK) {
        LOG_ERROR ("Failed to write data to SPI bus");
        status = eSTATUS_FAIL;
    }

    SPIEndOperation (pBus);
    return status;
}

eSTATUS_t
SPIWriteRead_Blocking (vSPIBus_t* pBus, eDEVICE_ID_t deviceId, uint8_t const* pTxData, uint8_t* pRxData, size_t size) {

    eSTATUS_t status = eSTATUS_OK;
    if (!SPIBeginOperation (pBus, deviceId)) {
        return eSTATUS_FAIL;
    }

    if (HAL_SPI_TransmitReceive (&pBus->handle, (uint8_t const*)pTxData, pRxData, size, HAL_MAX_DELAY) != HAL_OK) {
        LOG_ERROR ("Failed to write/read data to/from SPI bus");
        status = eSTATUS_FAIL;
    }

    SPIEndOperation (pBus);
    return status;
}

eSTATUS_t SPITransactions_Blocking (vSPIBus_t* pBus, eDEVICE_ID_t deviceId, BusTransaction_t* pTransactions, size_t nTransactions) {

    eSTATUS_t status            = eSTATUS_OK;
    HAL_StatusTypeDef halStatus = HAL_OK;

    if (!SPIBeginOperation (pBus, deviceId)) {
        return eSTATUS_FAIL;
    }

    for (size_t i = 0; i < nTransactions; ++i) {

        BusTransaction_t* pTrans = &pTransactions[i];
        uint8_t const* pTxData   = pTrans->pTxData;
        uint8_t* pRxData         = pTrans->pRxData;
        size_t txSize            = pTrans->txSize;
        size_t rxSize            = pTrans->rxSize;

        if (pTxData == NULL && pRxData == NULL) {
            LOG_ERROR ("Invalid SPI transaction segment, both pTxData and pRxData are NULL");
            status = eSTATUS_FAIL;
            break;
        }

        if (txSize == 0U && rxSize == 0U) {
            LOG_ERROR ("Invalid SPI transaction segment, both txSize and rxSize are 0");
            status = eSTATUS_FAIL;
            break;
        }

        if (pRxData != NULL && pTxData != NULL) {

            if (rxSize != txSize) {
                status = eSTATUS_FAIL;
                break;
            }

            halStatus = HAL_SPI_TransmitReceive (&pBus->handle, pTxData, pRxData, rxSize, HAL_MAX_DELAY);
            if (halStatus != HAL_OK) {
                LOG_ERROR ("Failed to write/read data to/from SPI bus");
                status = eSTATUS_FAIL;
                break;
            }
        } else if (pRxData != NULL) {

            halStatus = HAL_SPI_Receive (&pBus->handle, pRxData, rxSize, HAL_MAX_DELAY);
            if (halStatus != HAL_OK) {
                LOG_ERROR ("Failed to read data from SPI bus");
                status = eSTATUS_FAIL;
                break;
            }
        } else if (pTxData != NULL) {

            halStatus = HAL_SPI_Transmit (&pBus->handle, pTxData, txSize, HAL_MAX_DELAY);
            if (halStatus != HAL_OK) {
                LOG_ERROR ("Failed to write data to SPI bus");
                status = eSTATUS_FAIL;
                break;
            }
        }
    }

    SPIEndOperation (pBus);
    return status;
}

eSTATUS_t SPI_READ_BLOCKING (void* pCtx, eDEVICE_ID_t deviceId, uint8_t* pData, size_t size) {
    return SPIRead_Blocking ((vSPIBus_t*)pCtx, deviceId, pData, size);
}

eSTATUS_t SPI_WRITE_BLOCKING (void* pCtx, eDEVICE_ID_t deviceId, uint8_t const* pData, size_t size) {
    return SPIWrite_Blocking ((vSPIBus_t*)pCtx, deviceId, pData, size);
}

eSTATUS_t
SPI_WRITE_READ_BLOCKING (void* pCtx, eDEVICE_ID_t deviceId, uint8_t const* pTxData, uint8_t* pRxData, size_t size) {
    return SPIWriteRead_Blocking ((vSPIBus_t*)pCtx, deviceId, pTxData, pRxData, size);
}

eSTATUS_t SPI_TRANSACTIONS_BLOCKING (void* pCtx, eDEVICE_ID_t deviceId, BusTransaction_t* pTransactions, size_t nTransactions) {
    return SPITransactions_Blocking ((vSPIBus_t*)pCtx, deviceId, pTransactions, nTransactions);
}
