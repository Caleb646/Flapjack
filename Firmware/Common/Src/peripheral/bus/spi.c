#include "peripheral/bus/spi.h"
#include "conf/board.h"
#include "conf/conf.h"
#include "core/core.h"
#include "core/log/logger.h"
#include "hal.h"
#include "mem/mem.h"
#include "peripheral/bus/bus_core.h"
#include "peripheral/gpio.h"

#include "target.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
// eSPI_1_BUS_ID
#define SPI_CREATE(INSTANCE, SPI_NAME)                             \
    {                                                              \
        .id = e##SPI_NAME##_BUS_ID, .handle = { 0 }, .hardware = { \
            .pInstance = (INSTANCE),                               \
            .pSck      = SPI_NAME##_SCK_GPIO_PORT,                 \
            .pMiso     = SPI_NAME##_MISO_GPIO_PORT,                \
            .pMosi     = SPI_NAME##_MOSI_GPIO_PORT,                \
            .sckPin    = SPI_NAME##_SCK_GPIO_PIN,                  \
            .misoPin   = SPI_NAME##_MISO_GPIO_PIN,                 \
            .mosiPin   = SPI_NAME##_MOSI_GPIO_PIN,                 \
            .af        = SPI_NAME##_AF                             \
        }                                                          \
    }

FJ_DEFINE_SHARED (SpiBus_t, g_SpiBuses[]) = {
#if defined(SPI_1_ENABLED) && SPI_1_ENABLED == 1U
    SPI_CREATE (SPI1, SPI_1),
#endif
#if defined(SPI_2_ENABLED) && SPI_2_ENABLED == 1U
    SPI_CREATE (SPI2, SPI_2),
#endif
#if defined(SPI_3_ENABLED) && SPI_3_ENABLED == 1U
    SPI_CREATE (SPI3, SPI_3),
#endif
#if defined(SPI_4_ENABLED) && SPI_4_ENABLED == 1U
    SPI_CREATE (SPI4, SPI_4),
#endif
#if defined(SPI_5_ENABLED) && SPI_5_ENABLED == 1U
    SPI_CREATE (SPI5, SPI_5),
#endif
};

FJ_DEFINE_SHARED (uint32_t, g_nSpiBuses) = sizeof (g_SpiBuses) / sizeof (g_SpiBuses[0]);

eSTATUS_t Spi_InitSystem (void) {

    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = { 0 };

    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SPI1;
    PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL;
    __HAL_RCC_SPI1_CLK_ENABLE ();
    HAL_RCCEx_PeriphCLKConfig (&PeriphClkInitStruct);

    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SPI2;
    PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL;
    __HAL_RCC_SPI2_CLK_ENABLE ();
    HAL_RCCEx_PeriphCLKConfig (&PeriphClkInitStruct);

    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SPI3;
    PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL;
    __HAL_RCC_SPI3_CLK_ENABLE ();
    HAL_RCCEx_PeriphCLKConfig (&PeriphClkInitStruct);

    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SPI4;
    PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI45CLKSOURCE_PCLK2;
    __HAL_RCC_SPI4_CLK_ENABLE ();
    HAL_RCCEx_PeriphCLKConfig (&PeriphClkInitStruct);

    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SPI5;
    PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI45CLKSOURCE_PCLK2;
    __HAL_RCC_SPI5_CLK_ENABLE ();
    HAL_RCCEx_PeriphCLKConfig (&PeriphClkInitStruct);

#if defined(SPI_2_ENABLED) && SPI_2_ENABLED == 1U
    GPIO_ENABLE_CLOCK (SPI_2_SCK_GPIO_PORT);
#endif

    for (uint32_t i = 0; i < g_nSpiBuses; ++i) {
        GPIO_InitTypeDef gpioInit = { 0 };
        gpioInit.Mode             = GPIO_MODE_AF_PP;
        gpioInit.Pull             = GPIO_NOPULL;
        gpioInit.Speed            = GPIO_SPEED_FREQ_VERY_HIGH;

        gpioInit.Alternate = g_SpiBuses[i].hardware.af;
        gpioInit.Pin       = g_SpiBuses[i].hardware.sckPin;
        GPIO_ENABLE_CLOCK (g_SpiBuses[i].hardware.pSck);
        HAL_GPIO_Init (g_SpiBuses[i].hardware.pSck, &gpioInit);

        gpioInit.Alternate = g_SpiBuses[i].hardware.af;
        gpioInit.Pin       = g_SpiBuses[i].hardware.misoPin;
        GPIO_ENABLE_CLOCK (g_SpiBuses[i].hardware.pMiso);
        HAL_GPIO_Init (g_SpiBuses[i].hardware.pMiso, &gpioInit);

        gpioInit.Alternate = g_SpiBuses[i].hardware.af;
        gpioInit.Pin       = g_SpiBuses[i].hardware.mosiPin;
        GPIO_ENABLE_CLOCK (g_SpiBuses[i].hardware.pMosi);
        HAL_GPIO_Init (g_SpiBuses[i].hardware.pMosi, &gpioInit);

        SPI_HandleTypeDef* pHandle = &g_SpiBuses[i].handle;
        pHandle->Instance          = g_SpiBuses[i].hardware.pInstance;
        pHandle->Init.Mode         = SPI_MODE_MASTER;
        pHandle->Init.Direction    = SPI_DIRECTION_2LINES;
        pHandle->Init.DataSize     = SPI_DATASIZE_8BIT;
        pHandle->Init.CLKPolarity  = SPI_POLARITY_LOW;
        pHandle->Init.CLKPhase     = SPI_PHASE_1EDGE;
        // Manage NSS manually
        pHandle->Init.NSS               = SPI_NSS_SOFT;
        pHandle->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
        pHandle->Init.FirstBit          = SPI_FIRSTBIT_MSB;
        pHandle->Init.TIMode            = SPI_TIMODE_DISABLE;
        pHandle->Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
        pHandle->Init.CRCPolynomial     = 0x0;
        // Manage NSS manually
        pHandle->Init.NSSPMode                   = SPI_NSS_PULSE_DISABLE;
        pHandle->Init.NSSPolarity                = SPI_NSS_POLARITY_LOW;
        pHandle->Init.FifoThreshold              = SPI_FIFO_THRESHOLD_01DATA;
        pHandle->Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
        pHandle->Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
        pHandle->Init.MasterSSIdleness           = SPI_MASTER_SS_IDLENESS_00CYCLE;
        pHandle->Init.MasterInterDataIdleness    = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
        pHandle->Init.MasterReceiverAutoSusp     = SPI_MASTER_RX_AUTOSUSP_DISABLE;
        pHandle->Init.MasterKeepIOState          = SPI_MASTER_KEEP_IO_STATE_DISABLE;
        pHandle->Init.IOSwap                     = SPI_IO_SWAP_DISABLE;
        if (HAL_SPI_Init (pHandle) != HAL_OK) {
            return eSTATUS_FAILURE;
        }
    }
    return eSTATUS_SUCCESS;
}

eSTATUS_t SpiDev_Init (eBUS_ID_t spiBusId, GPIO_TypeDef* pNssPort, uint16_t nssPin, SpiDev_t* pOutDev) {

    SpiBus_t* pBus = NULL;
    for (uint32_t i = 0; i < g_nSpiBuses; ++i) {
        if (g_SpiBuses[i].id == spiBusId) {
            pBus = &g_SpiBuses[i];
            break;
        }
    }

    if (!pBus) {
        LOG_ERROR ("Failed to find SPI bus with ID [%u]", spiBusId);
        return eSTATUS_FAILURE;
    }

    GPIO_InitTypeDef gpioInit = { 0 };
    gpioInit.Mode             = GPIO_MODE_OUTPUT_PP;
    gpioInit.Pull             = GPIO_NOPULL;
    gpioInit.Speed            = GPIO_SPEED_FREQ_VERY_HIGH;
    gpioInit.Pin              = nssPin;
    GPIO_ENABLE_CLOCK (pNssPort);
    HAL_GPIO_Init (pNssPort, &gpioInit);
    HAL_GPIO_WritePin (pNssPort, nssPin, GPIO_PIN_SET);

    pOutDev->pBus     = pBus;
    pOutDev->pNssPort = pNssPort;
    pOutDev->nssPin   = nssPin;

    return eSTATUS_SUCCESS;
}

eSTATUS_t SpiDev_Write (SpiDev_t* pDev, uint8_t* pData, uint16_t size) {

    HAL_GPIO_WritePin (pDev->pNssPort, pDev->nssPin, GPIO_PIN_RESET);
    HAL_StatusTypeDef status = HAL_SPI_Transmit (&pDev->pBus->handle, pData, size, HAL_MAX_DELAY);
    HAL_GPIO_WritePin (pDev->pNssPort, pDev->nssPin, GPIO_PIN_SET);

    return (status == HAL_OK) ? eSTATUS_SUCCESS : eSTATUS_FAILURE;
}

eSTATUS_t SpiDev_WriteRegister (SpiDev_t* pDev, uint8_t reg, uint8_t* pData, uint16_t size) {

    HAL_GPIO_WritePin (pDev->pNssPort, pDev->nssPin, GPIO_PIN_RESET);
    reg &= 0x7FU; // clear read bit
    HAL_StatusTypeDef status  = HAL_SPI_Transmit (&pDev->pBus->handle, &reg, 1U, HAL_MAX_DELAY);
    HAL_StatusTypeDef status2 = HAL_SPI_Transmit (&pDev->pBus->handle, pData, size, HAL_MAX_DELAY);
    HAL_GPIO_WritePin (pDev->pNssPort, pDev->nssPin, GPIO_PIN_SET);

    return (status == HAL_OK && status2 == HAL_OK) ? eSTATUS_SUCCESS : eSTATUS_FAILURE;
}

eSTATUS_t SpiDev_ReadRegister (SpiDev_t* pDev, uint8_t reg, uint8_t* pOutData, uint16_t size) {

    HAL_GPIO_WritePin (pDev->pNssPort, pDev->nssPin, GPIO_PIN_RESET);
    reg |= 0x80U; // set read bit
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive (&pDev->pBus->handle, &reg, NULL, 1U, HAL_MAX_DELAY);
    HAL_StatusTypeDef status2 =
    HAL_SPI_TransmitReceive (&pDev->pBus->handle, NULL, pOutData, size, HAL_MAX_DELAY);
    HAL_GPIO_WritePin (pDev->pNssPort, pDev->nssPin, GPIO_PIN_SET);

    return (status == HAL_OK && status2 == HAL_OK) ? eSTATUS_SUCCESS : eSTATUS_FAILURE;
}

#define SPI_VALID(pBUS) ((pBUS) != NULL && (pBUS)->isInitialized == true)

static SHARED_MEM_SECTION SPIBus_t gSPIBusses[SPI_MAX_BUSES] = { 0 };

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

static INLINE vIO_t* SPIGetGPIOByDeviceId (vSPIBus_t* pBus, eDEVICE_ID_t deviceId) {

    for (uint32_t i = 0; i < pBus->nDevices; ++i) {

        if (pBus->connectedDevices.ids[i] == deviceId) {
            return pBus->connectedDevices.gpios[i];
        }
    }
    return NULL;
}

static INLINE bool SPIBeginOperation (vSPIBus_t* pBus, eDEVICE_ID_t deviceId) {

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

static INLINE void SPIEndOperation (vSPIBus_t* pBus) {

    if (pBus->activeOperation.pNss == NULL) {
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
            *(pSTATUS) = eSTATUS_FAILURE;                                                \
        }                                                                                \
        *(pSTATUS) = eSTATUS_SUCCESS;                                                    \
    } while (0)

static eSTATUS_t SPIClockInit (vSPIBus_t* pBus, SPIInitConf_t conf) {

    eBUS_ID_t busId  = pBus->busId;
    eSTATUS_t status = eSTATUS_SUCCESS;
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
    return eSTATUS_FAILURE;
}

static eSTATUS_t SPIInitGPIO (vSPIBus_t* pBus, SPIInitConf_t conf) {

    eSTATUS_t status = eSTATUS_SUCCESS;

    DeviceBoardConf_t device = conf.deviceBoardConf;
    /* Let the first device to initialize the spi bus be the owner of all of nss gpio pins */
    eDEVICE_ID_t firstDeviceId = device.deviceId;

    SPIBoardConf_t bus                 = conf.busBoardConf.SPIBoardConf;
    SPIDeviceMapping_t const* pDevices = bus.pConnectedDevices;
    uint8_t nDevices                   = bus.numConnectedDevices;
    GPIOBoardConf_t const* pSck        = bus.pSckBoardConf;
    GPIOBoardConf_t const* pMiso       = bus.pMisoBoardConf;
    GPIOBoardConf_t const* pMosi       = bus.pMosiBoardConf;

    if (pDevices == NULL || nDevices == 0 || nDevices > SPI_MAX_DEVICES_PER_BUS) {
        LOG_ERROR ("No or Beyond Max SPI devices configured for bus");
        return eSTATUS_FAILURE;
    }

    if (pSck == NULL || pMiso == NULL || pMosi == NULL) {
        LOG_ERROR ("SPI bus missing required pin configuration");
        return eSTATUS_FAILURE;
    }

    GPIO_INIT (&status, firstDeviceId, *pSck);
    if (STATUS_FAIL (status)) {
        LOG_ERROR ("Failed to initialize SPI SCK GPIO");
        return status;
    }

    GPIO_INIT (&status, firstDeviceId, *pMiso);
    if (STATUS_FAIL (status)) {
        LOG_ERROR ("Failed to initialize SPI MISO GPIO");
        return status;
    }

    GPIO_INIT (&status, firstDeviceId, *pMosi);
    if (STATUS_FAIL (status)) {
        LOG_ERROR ("Failed to initialize SPI MOSI GPIO");
        return status;
    }

    for (uint32_t i = 0; i < nDevices; ++i) {

        eDEVICE_ID_t deviceId       = pDevices[i].deviceId;
        GPIOBoardConf_t const* pNss = pDevices[i].pNssBoardConf;
        if (pNss == NULL) {
            LOG_ERROR ("SPI device missing NSS pin configuration");
            return eSTATUS_FAILURE;
        }

        GPIO_INIT (&status, firstDeviceId, *pNss);
        if (STATUS_FAIL (status)) {
            LOG_ERROR_IF (STATUS_FAIL (status), "Failed to initialize SPI NSS GPIO");
            return status;
        }

        pBus->connectedDevices.gpios[i] = GPIOGetIOfromId (pNss->id);
        pBus->connectedDevices.ids[i]   = deviceId;
    }
    pBus->nDevices = nDevices;
    return status;
}

vSPIBus_t* SPIGetBusById (eBUS_ID_t busId) {

    uint32_t busIndex = SPI_BUS_ID_TO_IDX (busId);
    if (BUS_ID_IS_SPI (busId) == false || busIndex >= SPI_MAX_BUSES) {
        LOG_ERROR ("Invalid SPI bus ID");
        return NULL;
    }
    return &gSPIBusses[busIndex];
}

eSTATUS_t SPIInit (SPIInitConf_t conf, vSPIBus_t* pOutBus) {

    DeviceBoardConf_t device = conf.deviceBoardConf;
    eDEVICE_ID_t deviceId    = device.deviceId;

    BusBoardConf_t bus = conf.busBoardConf;
    eBUS_ID_t busId    = bus.busId;
    RETURN_IF (BUS_ID_IS_SPI (busId) == false, eSTATUS_FAILURE, "Invalid SPI bus ID");

    SPIBoardConf_t spi = bus.SPIBoardConf;
    uint16_t speedKHz  = spi.speedKHz;
    (void)speedKHz;
    LOG_INFO ("Initializing SPI bus");

    vSPIBus_t* pBus = SPIGetBusById (busId);
    if (pOutBus != NULL) {
        pBus = pOutBus;
    }
    RETURN_IF_NULL (pBus, eSTATUS_FAILURE, "Failed to get SPI bus by ID");

    /* Let the first device to initialize the spi bus setup all of the given nss GPIO pins. */
    if (pBus->isInitialized == true) {
        LOG_WARN ("SPI bus %u is already initialized", busId);
        return eSTATUS_SUCCESS;
    }

    memset (pBus, 0, sizeof (vSPIBus_t));
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

    if (SPIClockInit (pBus, conf) != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to initialize SPI clock");
        goto error;
    }

    if (HAL_SPI_Init (&pBus->handle) != HAL_OK) {
        LOG_ERROR ("Failed to initialize HAL for SPI bus");
        goto error;
    }

    if (SPIInitGPIO (pBus, conf) != eSTATUS_SUCCESS) {
        LOG_ERROR ("Failed to initialize SPI GPIO data pins");
        goto error;
    }

    pBus->isInitialized = true;
    return eSTATUS_SUCCESS;

error:
    memset (pBus, 0, sizeof (vSPIBus_t));
    return eSTATUS_FAILURE;
}

eSTATUS_t SPIRead_Blocking (vSPIBus_t* pBus, eDEVICE_ID_t deviceId, uint8_t* pData, size_t size) {

    eSTATUS_t status = eSTATUS_SUCCESS;
    if (SPIBeginOperation (pBus, deviceId) == false) {
        return eSTATUS_FAILURE;
    }

    if (HAL_SPI_Receive (&pBus->handle, pData, size, HAL_MAX_DELAY) != HAL_OK) {
        LOG_ERROR ("Failed to read data from SPI bus");
        status = eSTATUS_FAILURE;
    }

    SPIEndOperation (pBus);
    return status;
}

eSTATUS_t SPIWrite_Blocking (vSPIBus_t* pBus, eDEVICE_ID_t deviceId, uint8_t const* pData, size_t size) {

    eSTATUS_t status = eSTATUS_SUCCESS;
    if (SPIBeginOperation (pBus, deviceId) == false) {
        return eSTATUS_FAILURE;
    }

    if (HAL_SPI_Transmit (&pBus->handle, (uint8_t*)pData, size, HAL_MAX_DELAY) != HAL_OK) {
        LOG_ERROR ("Failed to write data to SPI bus");
        status = eSTATUS_FAILURE;
    }

    SPIEndOperation (pBus);
    return status;
}

eSTATUS_t
SPIWriteRead_Blocking (vSPIBus_t* pBus, eDEVICE_ID_t deviceId, uint8_t const* pTxData, uint8_t* pRxData, size_t size) {

    eSTATUS_t status = eSTATUS_SUCCESS;
    if (SPIBeginOperation (pBus, deviceId) == false) {
        return eSTATUS_FAILURE;
    }

    if (HAL_SPI_TransmitReceive (&pBus->handle, (uint8_t*)pTxData, pRxData, size, HAL_MAX_DELAY) != HAL_OK) {
        LOG_ERROR ("Failed to write/read data to/from SPI bus");
        status = eSTATUS_FAILURE;
    }

    SPIEndOperation (pBus);
    return status;
}

eSTATUS_t SPITransactions_Blocking (vSPIBus_t* pBus, eDEVICE_ID_t deviceId, BusTransaction_t* pTransactions, size_t nTransactions) {

    eSTATUS_t status            = eSTATUS_SUCCESS;
    HAL_StatusTypeDef halStatus = HAL_OK;

    if (SPIBeginOperation (pBus, deviceId) == false) {
        return eSTATUS_FAILURE;
    }

    for (size_t i = 0; i < nTransactions; ++i) {

        BusTransaction_t* pTrans = &pTransactions[i];
        uint8_t const* pTxData   = pTrans->pTxData;
        uint8_t* pRxData         = pTrans->pRxData;
        size_t txSize            = pTrans->txSize;
        size_t rxSize            = pTrans->rxSize;

        if (pTxData == NULL && pRxData == NULL) {
            LOG_ERROR ("Invalid SPI transaction segment, both pTxData and pRxData are NULL");
            status = eSTATUS_FAILURE;
            break;
        }

        if (txSize == 0U && rxSize == 0U) {
            LOG_ERROR ("Invalid SPI transaction segment, both txSize and rxSize are 0");
            status = eSTATUS_FAILURE;
            break;
        }

        if (pRxData != NULL && pTxData != NULL) {

            if (rxSize != txSize) {
                status = eSTATUS_FAILURE;
                break;
            }

            halStatus = HAL_SPI_TransmitReceive (&pBus->handle, pTxData, pRxData, rxSize, HAL_MAX_DELAY);
            if (halStatus != HAL_OK) {
                LOG_ERROR ("Failed to write/read data to/from SPI bus");
                status = eSTATUS_FAILURE;
                break;
            }
        } else if (pRxData != NULL) {

            halStatus = HAL_SPI_Receive (&pBus->handle, pRxData, rxSize, HAL_MAX_DELAY);
            if (halStatus != HAL_OK) {
                LOG_ERROR ("Failed to read data from SPI bus");
                status = eSTATUS_FAILURE;
                break;
            }
        } else if (pTxData != NULL) {

            halStatus = HAL_SPI_Transmit (&pBus->handle, (uint8_t*)pTxData, txSize, HAL_MAX_DELAY);
            if (halStatus != HAL_OK) {
                LOG_ERROR ("Failed to write data to SPI bus");
                status = eSTATUS_FAILURE;
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
