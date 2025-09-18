#include "periphs/spi.h"
#include "common.h"
#include "hal.h"
#include "log/logger.h"
#include "periphs/gpio.h"


#define IS_BUS_VALID(pBus) (pBus != NULL && pBus->isInitialized == TRUE)

static SPIBus_t gSPIBusses[eSPI_BUS_ID_MAX] = { 0 };

static SPI_TypeDef* SPIGetInstanceById (eSPI_BUS_ID_t busId) {

    switch (busId) {
    case eSPI_1_BUS_ID: return SPI1;
    case eSPI_2_BUS_ID: return SPI2;
    case eSPI_3_BUS_ID: return SPI3;
    case eSPI_4_BUS_ID: return SPI4;
    case eSPI_5_BUS_ID: return SPI5;
    default: LOG_ERROR ("Invalid SPI bus ID"); return NULL;
    }
}

static SPIBus_t* SPIGetBusById (eSPI_BUS_ID_t busId) {

    if (busId >= eSPI_BUS_ID_MAX) {
        LOG_ERROR ("Invalid SPI bus ID");
        return NULL;
    }
    return &gSPIBusses[busId];
}

static GPIOSPI_t* SPIGetGPIOByDeviceId (SPIBus_t* pBus, eDEVICE_ID_t deviceId) {

    if (pBus == NULL) {
        LOG_ERROR ("Failed to get SPI bus by ID");
        return NULL;
    }
    for (uint32_t i = 0; i < pBus->nDevices; i++) {
        if (pBus->deviceIds[i] == deviceId) {
            return &pBus->gpio[i];
        }
    }
    return NULL;
}

static BOOL_t SPIAddDevice2Bus (SPIBus_t* pBus, eDEVICE_ID_t deviceId) {

    if (pBus == NULL) {
        LOG_ERROR ("Failed to get SPI bus by ID");
        return FALSE;
    }

    if (pBus->nDevices >= SPI_MAX_DEVICES_PER_BUS) {
        LOG_ERROR ("SPI bus has reached maximum number of devices");
        return FALSE;
    }

    pBus->deviceIds[pBus->nDevices] = deviceId;
    pBus->nDevices++;
    return TRUE;
}

static eSTATUS_t SPIClockInit (eSPI_BUS_ID_t busId) {

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

eSTATUS_t SPIInit (SPIInitConf_t const* pConf) {

    if (pConf == NULL) {
        LOG_ERROR ("Invalid SPI configuration");
        return eSTATUS_FAILURE;
    }

    SPIBus_t* pBus = SPIGetBusById (pConf->busId);
    if (pBus == NULL) {
        LOG_ERROR ("Failed to get SPI bus by ID");
        return eSTATUS_FAILURE;
    }

    if (pBus->isInitialized == TRUE) {

        if (pBus->speed != pConf->speed) {
            LOG_ERROR ("Cannot add another device to an SPI bus with a different speed");
            return eSTATUS_FAILURE;
        }

        if (SPIAddDevice2Bus (pBus, pConf->deviceId) != TRUE) {
            LOG_ERROR ("Failed to add additional device to SPI bus");
            return eSTATUS_FAILURE;
        }

        if (GPIOInitSPI (pBus->handle.Instance, &pBus->gpio[pBus->nDevices - 1]) !=
            eSTATUS_SUCCESS) {
            LOG_ERROR ("Failed to initialize GPIO for additional SPI device");
            return eSTATUS_FAILURE;
        }
        return eSTATUS_SUCCESS;
    }

    memset (pBus, 0, sizeof (SPIBus_t));
    pBus->busId                   = pConf->busId;
    pBus->speed                   = pConf->speed;
    pBus->isInitialized           = TRUE;
    pBus->handle.Instance         = SPIGetInstanceById (pConf->busId);
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

    if (HAL_SPI_Init (&pBus->handle) != HAL_OK) {
        LOG_ERROR ("Failed to initialize HAL for SPI bus");
        goto error;
    }

    if (SPIClockInit (pBus->busId) != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to initialize SPI clock");
        goto error;
    }

    if (GPIOInitSPI (pBus->handle.Instance, &pBus->gpio[0]) != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to initialize SPI GPIO");
        goto error;
    }

    if (SPIAddDevice2Bus (pBus, pConf->deviceId) != TRUE) {
        LOG_ERROR ("Failed to add device to SPI bus");
        goto error;
    }
    return eSTATUS_SUCCESS;

error:
    memset (pBus, 0, sizeof (SPIBus_t));
    return eSTATUS_FAILURE;
}

eSTATUS_t
SPIRead_Blocking (eSPI_BUS_ID_t busId, eDEVICE_ID_t deviceId, uint8_t* pData, uint16_t size) {

    SPIBus_t* pBus = SPIGetBusById (busId);
    if (IS_BUS_VALID (pBus) == FALSE) {
        LOG_ERROR ("Failed to get SPI bus by ID");
        return eSTATUS_FAILURE;
    }

    GPIOSPI_t* pGPIO = SPIGetGPIOByDeviceId (pBus, deviceId);
    if (pGPIO == NULL) {
        LOG_ERROR ("Failed to get GPIO for SPI device ID");
        return eSTATUS_FAILURE;
    }

    HAL_GPIO_WritePin (pGPIO->pNSSPort, pGPIO->nssPin, GPIO_PIN_RESET);
    if (HAL_SPI_Receive (&pBus->handle, pData, size, HAL_MAX_DELAY) != HAL_OK) {
        LOG_ERROR ("Failed to read data from SPI bus");
        return eSTATUS_FAILURE;
    }
    HAL_GPIO_WritePin (pGPIO->pNSSPort, pGPIO->nssPin, GPIO_PIN_SET);
    return eSTATUS_SUCCESS;
}

eSTATUS_t
SPIWrite_Blocking (eSPI_BUS_ID_t busId, eDEVICE_ID_t deviceId, uint8_t const* pData, uint16_t size) {

    SPIBus_t* pBus = SPIGetBusById (busId);
    if (IS_BUS_VALID (pBus) == FALSE) {
        LOG_ERROR ("Failed to get SPI bus by ID");
        return eSTATUS_FAILURE;
    }

    GPIOSPI_t* pGPIO = SPIGetGPIOByDeviceId (pBus, deviceId);
    if (pGPIO == NULL) {
        LOG_ERROR ("Failed to get GPIO for SPI device ID");
        return eSTATUS_FAILURE;
    }

    HAL_GPIO_WritePin (pGPIO->pNSSPort, pGPIO->nssPin, GPIO_PIN_RESET);
    if (HAL_SPI_Transmit (&pBus->handle, (uint8_t*)pData, size, HAL_MAX_DELAY) != HAL_OK) {
        LOG_ERROR ("Failed to write data to SPI bus");
        return eSTATUS_FAILURE;
    }
    HAL_GPIO_WritePin (pGPIO->pNSSPort, pGPIO->nssPin, GPIO_PIN_SET);
    return eSTATUS_SUCCESS;
}

eSTATUS_t SPIWriteRead_Blocking (
eSPI_BUS_ID_t busId,
eDEVICE_ID_t deviceId,
uint8_t const* pTxData,
uint8_t* pRxData,
uint16_t size
) {

    SPIBus_t* pBus = SPIGetBusById (busId);
    if (IS_BUS_VALID (pBus) == FALSE) {
        LOG_ERROR ("Failed to get SPI bus by ID");
        return eSTATUS_FAILURE;
    }

    GPIOSPI_t* pGPIO = SPIGetGPIOByDeviceId (pBus, deviceId);
    if (pGPIO == NULL) {
        LOG_ERROR ("Failed to get GPIO for SPI device ID");
        return eSTATUS_FAILURE;
    }

    HAL_GPIO_WritePin (pGPIO->pNSSPort, pGPIO->nssPin, GPIO_PIN_RESET);
    if (HAL_SPI_TransmitReceive (&pBus->handle, (uint8_t*)pTxData, pRxData, size, HAL_MAX_DELAY) !=
        HAL_OK) {
        LOG_ERROR ("Failed to write/read data to/from SPI bus");
        return eSTATUS_FAILURE;
    }
    HAL_GPIO_WritePin (pGPIO->pNSSPort, pGPIO->nssPin, GPIO_PIN_SET);
    return eSTATUS_SUCCESS;
}
