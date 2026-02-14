#include "hal.h"
#include "target.h"

#include "core/core.h"

#include "drivers/bus/spi.h"

#include "drivers/io/gpio.h"

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

        GPIO_ENABLE_CLOCK (g_SpiBuses[i].hardware.pSck);
        GPIO_ENABLE_CLOCK (g_SpiBuses[i].hardware.pMiso);
        GPIO_ENABLE_CLOCK (g_SpiBuses[i].hardware.pMosi);

        GPIO_InitTypeDef gpioInit = { 0 };
        gpioInit.Mode             = GPIO_MODE_AF_PP;
        gpioInit.Pull             = GPIO_NOPULL;
        gpioInit.Speed            = GPIO_SPEED_FREQ_VERY_HIGH;
        gpioInit.Alternate        = g_SpiBuses[i].hardware.af;

        gpioInit.Pin = g_SpiBuses[i].hardware.sckPin;
        HAL_GPIO_Init (g_SpiBuses[i].hardware.pSck, &gpioInit);

        gpioInit.Pin = g_SpiBuses[i].hardware.misoPin;
        HAL_GPIO_Init (g_SpiBuses[i].hardware.pMiso, &gpioInit);

        gpioInit.Pin = g_SpiBuses[i].hardware.mosiPin;
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

SpiBus_t* Spi_GetBusById (eBUS_ID_t busId) {

    for (uint32_t i = 0; i < g_nSpiBuses; ++i) {
        if (g_SpiBuses[i].id == busId) {
            return &g_SpiBuses[i];
        }
    }
    return NULL;
}

eSTATUS_t SpiDev_Init (SpiDev_t* pOutDev) {

    if (!pOutDev) {
        return eSTATUS_FAILURE;
    }

    SpiBus_t* pBus = Spi_GetBusById (pOutDev->cfg.busId);
    if (!pBus) {
        return eSTATUS_FAILURE;
    }

    GPIO_TypeDef* pNssPort    = pOutDev->cfg.pNssPort;
    uint16_t nssPin           = pOutDev->cfg.nssPin;
    GPIO_InitTypeDef gpioInit = { 0 };
    gpioInit.Mode             = GPIO_MODE_OUTPUT_PP;
    gpioInit.Pull             = GPIO_NOPULL;
    gpioInit.Speed            = GPIO_SPEED_FREQ_VERY_HIGH;
    gpioInit.Pin              = nssPin;
    GPIO_ENABLE_CLOCK (pNssPort);
    HAL_GPIO_Init (pNssPort, &gpioInit);
    HAL_GPIO_WritePin (pNssPort, nssPin, GPIO_PIN_SET);

    pOutDev->pBus = pBus;
    return eSTATUS_SUCCESS;
}

eSTATUS_t SpiDev_Write (SpiDev_t* pDev, uint8_t const* pData, uint16_t size) {

    HAL_GPIO_WritePin (pDev->cfg.pNssPort, pDev->cfg.nssPin, GPIO_PIN_RESET);
    HAL_StatusTypeDef status = HAL_SPI_Transmit (&pDev->pBus->handle, pData, size, HAL_MAX_DELAY);
    HAL_GPIO_WritePin (pDev->cfg.pNssPort, pDev->cfg.nssPin, GPIO_PIN_SET);

    return (status == HAL_OK) ? eSTATUS_SUCCESS : eSTATUS_FAILURE;
}

eSTATUS_t SpiDev_WriteRegister (SpiDev_t* pDev, uint8_t reg, uint8_t const* pData, uint16_t size) {

    HAL_GPIO_WritePin (pDev->cfg.pNssPort, pDev->cfg.nssPin, GPIO_PIN_RESET);
    reg &= 0x7FU; // clear read bit
    HAL_StatusTypeDef status  = HAL_SPI_Transmit (&pDev->pBus->handle, &reg, 1U, HAL_MAX_DELAY);
    HAL_StatusTypeDef status2 = HAL_SPI_Transmit (&pDev->pBus->handle, pData, size, HAL_MAX_DELAY);
    HAL_GPIO_WritePin (pDev->cfg.pNssPort, pDev->cfg.nssPin, GPIO_PIN_SET);

    return (status == HAL_OK && status2 == HAL_OK) ? eSTATUS_SUCCESS : eSTATUS_FAILURE;
}

eSTATUS_t SpiDev_ReadRegister (SpiDev_t* pDev, uint8_t reg, uint8_t* pOutData, uint16_t size) {

    HAL_GPIO_WritePin (pDev->cfg.pNssPort, pDev->cfg.nssPin, GPIO_PIN_RESET);
    reg |= 0x80U; // set read bit
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive (&pDev->pBus->handle, &reg, NULL, 1U, HAL_MAX_DELAY);
    HAL_StatusTypeDef status2 =
    HAL_SPI_TransmitReceive (&pDev->pBus->handle, NULL, pOutData, size, HAL_MAX_DELAY);
    HAL_GPIO_WritePin (pDev->cfg.pNssPort, pDev->cfg.nssPin, GPIO_PIN_SET);

    return (status == HAL_OK && status2 == HAL_OK) ? eSTATUS_SUCCESS : eSTATUS_FAILURE;
}

eSTATUS_t SpiDev_WriteRead (SpiDev_t* pDev, uint8_t const* pTxData, uint8_t* pRxData, uint16_t size) {

    HAL_GPIO_WritePin (pDev->cfg.pNssPort, pDev->cfg.nssPin, GPIO_PIN_RESET);
    HAL_StatusTypeDef status =
    HAL_SPI_TransmitReceive (&pDev->pBus->handle, pTxData, pRxData, size, HAL_MAX_DELAY);
    HAL_GPIO_WritePin (pDev->cfg.pNssPort, pDev->cfg.nssPin, GPIO_PIN_SET);
    return (status == HAL_OK) ? eSTATUS_SUCCESS : eSTATUS_FAILURE;
}

eSTATUS_t SpiDev_Transactions (SpiDev_t* pDev, SpiDevTransaction_t* pTransactions, uint32_t nTransactions) {

    HAL_GPIO_WritePin (pDev->cfg.pNssPort, pDev->cfg.nssPin, GPIO_PIN_RESET);
    for (uint32_t i = 0; i < nTransactions; ++i) {
        SpiDevTransaction_t* pTransaction = &pTransactions[i];
        HAL_SPI_TransmitReceive (
        &pDev->pBus->handle,
        pTransaction->pTxData,
        pTransaction->pRxData,
        pTransaction->size,
        HAL_MAX_DELAY
        );
    }
    HAL_GPIO_WritePin (pDev->cfg.pNssPort, pDev->cfg.nssPin, GPIO_PIN_SET);
    return eSTATUS_SUCCESS;
}
