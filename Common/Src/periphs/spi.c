#include "periphs/spi.h"
#include "common.h"
#include "conf/conf.h"
#include "hal.h"
#include "log/logger.h"
#include "periphs/gpio.h"

#define IS_BUS_VALID(pBus) (pBus != NULL && pBus->isInitialized == TRUE)
#define SPI_BEGIN(busId)                                    \
    SPIBus_t* pBus = SPIGetBusById (busId);                 \
    if (IS_BUS_VALID (pBus) == FALSE) {                     \
        LOG_ERROR ("Failed to get SPI bus by ID");          \
        return eSTATUS_FAILURE;                             \
    }                                                       \
    IO_t* pGPIO = SPIGetGPIOByDeviceId (pBus, deviceId);    \
    if (pGPIO == NULL) {                                    \
        LOG_ERROR ("Failed to get GPIO for SPI device ID"); \
        return eSTATUS_FAILURE;                             \
    }                                                       \
    HAL_GPIO_WritePin (pGPIO->pPort, pGPIO->pin, GPIO_PIN_RESET)

#define SPI_END() \
    HAL_GPIO_WritePin (pGPIO->pPort, pGPIO->pin, GPIO_PIN_SET)

static SPIBus_t gSPIBusses[eSPI_BUS_ID_MAX] = { 0 };

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

static SPIBus_t* SPIGetBusById (eBUS_ID_t busId) {

    if (BUS_ID_IS_SPI (busId) == FALSE) {
        LOG_ERROR ("Invalid SPI bus ID");
        return NULL;
    }
    return &gSPIBusses[busId];
}

static IO_t* SPIGetGPIOByDeviceId (SPIBus_t* pBus, eDEVICE_ID_t deviceId) {

    if (pBus == NULL) {
        LOG_ERROR ("Failed to get SPI bus by ID");
        return NULL;
    }
    for (uint32_t i = 0; i < pBus->nDevices; i++) {
        if (pBus->deviceIds[i] == deviceId) {
            return GPIOGetIOfromId (pBus->nss[i]);
        }
    }
    return NULL;
}

static BOOL_t SPIAddDevice2Bus (SPIBus_t* pBus, eGPIO_ID_t nssId, eDEVICE_ID_t deviceId) {

    if (pBus == NULL) {
        LOG_ERROR ("Failed to get SPI bus by ID");
        return FALSE;
    }

    if (pBus->nDevices >= SPI_MAX_DEVICES_PER_BUS) {
        LOG_ERROR ("SPI bus has reached maximum number of devices");
        return FALSE;
    }

    eSTATUS_t status = eSTATUS_SUCCESS;
    GPIO_INIT_SPI_SOFTWARE_NSS_ONLY (&status, deviceId, nssId);

    pBus->nss[pBus->nDevices]       = nssId;
    pBus->deviceIds[pBus->nDevices] = deviceId;
    pBus->nDevices++;
    return TRUE;
}

#define SPI_INIT_CLOCK(pSTATUS, BUS_ID, RCC_PERIPH_CLK_SELECTION, RCC_SPI_CLK_SELECTION) \
    do {                                                                                 \
        RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = { 0 };                            \
        PeriphClkInitStruct.PeriphClockSelection = (RCC_PERIPH_CLK_SELECTION);           \
        PeriphClkInitStruct.Spi123ClockSelection = (RCC_SPI_CLK_SELECTION);              \
        if ((BUS_ID) > eSPI_3_BUS_ID) {                                                  \
            PeriphClkInitStruct.Spi123ClockSelection = 0;                                \
            PeriphClkInitStruct.Spi45ClockSelection = (RCC_SPI_CLK_SELECTION);           \
        }                                                                                \
        if (HAL_RCCEx_PeriphCLKConfig (&PeriphClkInitStruct) != HAL_OK) {                \
            LOG_ERROR ("Failed to configure SPI [%u] clock", (BUS_ID));                  \
            *(pSTATUS) = eSTATUS_FAILURE;                                                \
        }                                                                                \
        *(pSTATUS) = eSTATUS_SUCCESS;                                                    \
    } while (0)

static eSTATUS_t
SPIClockInit (SPIBus_t* pBus, SPIInitConf_t conf, SPIBoardConf_t busBoardConf) {

    eBUS_ID_t busId  = pBus->busId;
    eSTATUS_t status = eSTATUS_SUCCESS;
    if (busId == eSPI_1_BUS_ID) {
        SPI_INIT_CLOCK (&status, eSPI_1_BUS_ID, RCC_PERIPHCLK_SPI1, RCC_SPI123CLKSOURCE_PLL);
        return status;
    }
    if (busId == eSPI_2_BUS_ID) {
        SPI_INIT_CLOCK (&status, eSPI_2_BUS_ID, RCC_PERIPHCLK_SPI2, RCC_SPI123CLKSOURCE_PLL);
        return status;
    }
    if (busId == eSPI_3_BUS_ID) {
        SPI_INIT_CLOCK (&status, eSPI_3_BUS_ID, RCC_PERIPHCLK_SPI3, RCC_SPI123CLKSOURCE_PLL);
        return status;
    }
    if (busId == eSPI_4_BUS_ID) {
        SPI_INIT_CLOCK (&status, eSPI_4_BUS_ID, RCC_PERIPHCLK_SPI4, RCC_SPI45CLKSOURCE_PCLK2);
        return status;
    }
    if (busId == eSPI_5_BUS_ID) {
        SPI_INIT_CLOCK (&status, eSPI_5_BUS_ID, RCC_PERIPHCLK_SPI5, RCC_SPI45CLKSOURCE_PCLK2);
        return status;
    }
    LOG_ERROR ("Unsupported SPI bus ID");
    return eSTATUS_FAILURE;
}

static eSTATUS_t
SPIInitGPIO (SPIBus_t* pBus, SPIInitConf_t conf, SPIBoardConf_t busBoardConf) {

    eSTATUS_t status = eSTATUS_SUCCESS;
    eBUS_ID_t busId  = busBoardConf.header.busId;
    GPIO_INIT_SPI_DATA_ONLY (
    &status,
    busId,
    busBoardConf.sckId,
    busBoardConf.misoId,
    busBoardConf.mosiId,
    busBoardConf.gpioAlternate
    );
    LOG_ERROR_IF (STATUS_FAIL (status), "Failed to initialize SPI GPIO");
    return status;
}

eSTATUS_t SPIInit (SPIInitConf_t conf, SPIBoardConf_t boardConf) {

    eDEVICE_ID_t deviceId = conf.deviceId;
    eGPIO_ID_t nssId      = conf.nssId;
    eBUS_ID_t busId       = boardConf.header.busId;
    uint16_t speedKHz     = boardConf.speedKHz;
    LOG_INFO ("Initializing SPI bus %u", busId);

    RETURN_IF_NOT (BUS_ID_IS_SPI (busId), eSTATUS_FAILURE, "Invalid SPI bus ID");

    SPIBus_t* pBus = SPIGetBusById (busId);
    RETURN_IF_NULL (pBus, eSTATUS_FAILURE, "Failed to get SPI bus by ID");

    if (pBus->isInitialized == TRUE) {

        if (SPIAddDevice2Bus (pBus, nssId, deviceId) != TRUE) {
            LOG_ERROR ("Failed to add additional device to SPI bus");
            return eSTATUS_FAILURE;
        }

        return eSTATUS_SUCCESS;
    }

    memset (pBus, 0, sizeof (SPIBus_t));
    pBus->busId                   = busId;
    pBus->deviceId                = deviceId;
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
    pBus->handle.Init.NSSPMode      = SPI_NSS_PULSE_DISABLE;
    pBus->handle.Init.NSSPolarity   = SPI_NSS_POLARITY_LOW;
    pBus->handle.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
    pBus->handle.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
    pBus->handle.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
    pBus->handle.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
    pBus->handle.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
    pBus->handle.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
    pBus->handle.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
    pBus->handle.Init.IOSwap            = SPI_IO_SWAP_DISABLE;

    if (SPIClockInit (pBus, conf, boardConf) != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to initialize SPI clock");
        goto error;
    }

    if (HAL_SPI_Init (&pBus->handle) != HAL_OK) {
        LOG_ERROR ("Failed to initialize HAL for SPI bus");
        goto error;
    }

    if (SPIInitGPIO (pBus, conf, boardConf) != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to initialize SPI GPIO data pins");
        goto error;
    }

    if (SPIAddDevice2Bus (pBus, nssId, deviceId) != TRUE) {
        LOG_ERROR ("Failed to add device to SPI bus");
        goto error;
    }

    pBus->isInitialized = TRUE;
    return eSTATUS_SUCCESS;

error:
    memset (pBus, 0, sizeof (SPIBus_t));
    return eSTATUS_FAILURE;
}

eSTATUS_t
SPIRead_Blocking (eBUS_ID_t busId, eDEVICE_ID_t deviceId, uint8_t* pData, uint16_t size) {

    eSTATUS_t status = eSTATUS_SUCCESS;
    SPI_BEGIN (busId);

    if (HAL_SPI_Receive (&pBus->handle, pData, size, HAL_MAX_DELAY) != HAL_OK) {
        LOG_ERROR ("Failed to read data from SPI bus");
        status = eSTATUS_FAILURE;
    }

    SPI_END ();
    return status;
}

eSTATUS_t
SPIWrite_Blocking (eBUS_ID_t busId, eDEVICE_ID_t deviceId, uint8_t const* pData, uint16_t size) {

    eSTATUS_t status = eSTATUS_SUCCESS;
    SPI_BEGIN (busId);

    if (HAL_SPI_Transmit (&pBus->handle, (uint8_t*)pData, size, HAL_MAX_DELAY) != HAL_OK) {
        LOG_ERROR ("Failed to write data to SPI bus");
        status = eSTATUS_FAILURE;
    }

    SPI_END ();
    return status;
}

eSTATUS_t
SPIWriteRead_Blocking (eBUS_ID_t busId, eDEVICE_ID_t deviceId, uint8_t const* pTxData, uint8_t* pRxData, uint16_t size) {

    eSTATUS_t status = eSTATUS_SUCCESS;
    SPI_BEGIN (busId);

    if (HAL_SPI_TransmitReceive (&pBus->handle, (uint8_t*)pTxData, pRxData, size, HAL_MAX_DELAY) !=
        HAL_OK) {
        LOG_ERROR ("Failed to write/read data to/from SPI bus");
        status = eSTATUS_FAILURE;
    }

    SPI_END ();
    return status;
}
