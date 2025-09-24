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

static eSTATUS_t SPIClockInit (eBUS_ID_t busId) {

    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = { 0 };
    if (busId == eSPI_1_BUS_ID) {

        PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SPI1;
        PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL;

        if (HAL_RCCEx_PeriphCLKConfig (&PeriphClkInitStruct) != HAL_OK) {
            LOG_ERROR ("Failed to configure SPI1 clock");
            return eSTATUS_FAILURE;
        }
    } else if (busId == eSPI_2_BUS_ID) {

        PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SPI2;
        PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL;

        if (HAL_RCCEx_PeriphCLKConfig (&PeriphClkInitStruct) != HAL_OK) {
            LOG_ERROR ("Failed to configure SPI2 clock");
            return eSTATUS_FAILURE;
        }
    } else if (busId == eSPI_3_BUS_ID) {

        PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SPI3;
        PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL;

        if (HAL_RCCEx_PeriphCLKConfig (&PeriphClkInitStruct) != HAL_OK) {
            LOG_ERROR ("Failed to configure SPI3 clock");
            return eSTATUS_FAILURE;
        }
    } else if (busId == eSPI_4_BUS_ID) {

        PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SPI4;
        PeriphClkInitStruct.Spi45ClockSelection = RCC_SPI45CLKSOURCE_PCLK2;

        if (HAL_RCCEx_PeriphCLKConfig (&PeriphClkInitStruct) != HAL_OK) {
            LOG_ERROR ("Failed to configure SPI4 clock");
            return eSTATUS_FAILURE;
        }
    } else if (busId == eSPI_5_BUS_ID) {

        PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SPI5;
        PeriphClkInitStruct.Spi45ClockSelection = RCC_SPI45CLKSOURCE_PCLK2;

        if (HAL_RCCEx_PeriphCLKConfig (&PeriphClkInitStruct) != HAL_OK) {
            LOG_ERROR ("Failed to configure SPI5 clock");
            return eSTATUS_FAILURE;
        }
    } else {
        LOG_ERROR ("Unsupported SPI bus ID");
        return eSTATUS_FAILURE;
    }
    return eSTATUS_SUCCESS;
}

static eSTATUS_t SPIInitAllGPIODataPins (eBUS_ID_t busId) {

    eSTATUS_t status = eSTATUS_SUCCESS;
    if (busId == eSPI_1_BUS_ID) {
        GPIO_INIT_SPI_DATA_ONLY (
        &status,
        eSPI_1_BUS_ID,
        eGPIO_PORTID_A | eGPIO_PINID_5, // SCK
        eGPIO_PORTID_A | eGPIO_PINID_6, // MISO
        eGPIO_PORTID_A | eGPIO_PINID_7, // MOSI
        GPIO_AF5_SPI1
        );

        if (status != eSTATUS_SUCCESS) {
            LOG_ERROR ("Failed to initialize SPI1 GPIO");
            return status;
        }
    } else if (busId == eSPI_3_BUS_ID) {
        GPIO_INIT_SPI_DATA_ONLY (
        &status,
        eSPI_3_BUS_ID,
        eGPIO_PORTID_C | eGPIO_PINID_10, // SCK
        eGPIO_PORTID_C | eGPIO_PINID_11, // MISO
        eGPIO_PORTID_C | eGPIO_PINID_12, // MOSI
        GPIO_AF6_SPI3
        );

        if (status != eSTATUS_SUCCESS) {
            LOG_ERROR ("Failed to initialize SPI3 GPIO");
            return status;
        }
    } else if (busId == eSPI_5_BUS_ID) {
        GPIO_INIT_SPI_DATA_ONLY (
        &status,
        eSPI_5_BUS_ID,
        eGPIO_PORTID_F | eGPIO_PINID_7, // SCK
        eGPIO_PORTID_F | eGPIO_PINID_8, // MISO
        eGPIO_PORTID_F | eGPIO_PINID_9, // MOSI
        GPIO_AF5_SPI5
        );
    }
    return eSTATUS_SUCCESS;
}

eSTATUS_t SPIInit (SPIInitConf_t conf) {

    if (BUS_ID_IS_SPI (conf.busId) == FALSE) {
        LOG_ERROR ("Invalid SPI bus ID");
        return eSTATUS_FAILURE;
    }

    SPIBus_t* pBus = SPIGetBusById (conf.busId);
    if (pBus == NULL) {
        LOG_ERROR ("Failed to get SPI bus by ID");
        return eSTATUS_FAILURE;
    }

    if (pBus->isInitialized == TRUE) {

        if (SPIAddDevice2Bus (pBus, conf.nssId, conf.deviceId) != TRUE) {
            LOG_ERROR ("Failed to add additional device to SPI bus");
            return eSTATUS_FAILURE;
        }

        return eSTATUS_SUCCESS;
    }

    memset (pBus, 0, sizeof (SPIBus_t));
    pBus->conf                    = conf;
    pBus->handle.Instance         = SPIGetInstanceById (conf.busId);
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

    if (SPIClockInit (conf.busId) != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to initialize SPI clock");
        goto error;
    }

    if (HAL_SPI_Init (&pBus->handle) != HAL_OK) {
        LOG_ERROR ("Failed to initialize HAL for SPI bus");
        goto error;
    }

    if (SPIInitGPIO (conf.busId) != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to initialize SPI GPIO data pins");
        goto error;
    }

    if (SPIAddDevice2Bus (pBus, conf.nssId, conf.deviceId) != TRUE) {
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
