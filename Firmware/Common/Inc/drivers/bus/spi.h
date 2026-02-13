#ifndef PERIPHS_SPI_H
#define PERIPHS_SPI_H

#include "core/core.h"
#include "hal.h"
#include "target.h"

#include "drivers/io/gpio.h"

#include <stdint.h>
#include <string.h>

#define SPI_MAX_DEVICES_PER_BUS 3U

typedef struct {
    SPI_TypeDef* pInstance;
    GPIO_TypeDef* pSck;
    GPIO_TypeDef* pMiso;
    GPIO_TypeDef* pMosi;
    uint16_t sckPin;
    uint16_t misoPin;
    uint16_t mosiPin;
    uint32_t af;
} SpiHardware_t;

typedef struct {
    eBUS_ID_t id;
    SPI_HandleTypeDef handle;
    SpiHardware_t hardware;
} SpiBus_t;

typedef struct {
    SpiBus_t* pBus;
    struct {
        eBUS_ID_t busId;
        GPIO_TypeDef* pNssPort;
        uint16_t nssPin;
    } cfg;
} SpiDev_t;

typedef struct {
    uint8_t const* pTxData;
    uint8_t* pRxData;
    uint32_t size;
} SpiDevTransaction_t;

FJ_DECLARE_SHARED (SpiBus_t, g_SpiBuses[]);
FJ_DECLARE_SHARED (uint32_t, g_nSpiBuses);

eSTATUS_t Spi_InitSystem (void);
eSTATUS_t SpiDev_Init (SpiDev_t* pOutDev);
eSTATUS_t SpiDev_Write (SpiDev_t* pDev, uint8_t const* pData, uint16_t size);
eSTATUS_t SpiDev_WriteRegister (SpiDev_t* pDev, uint8_t reg, uint8_t const* pData, uint16_t size);
eSTATUS_t SpiDev_ReadRegister (SpiDev_t* pDev, uint8_t reg, uint8_t* pOutData, uint16_t size);
eSTATUS_t SpiDev_WriteRead (SpiDev_t* pDev, uint8_t const* pTxData, uint8_t* pRxData, uint16_t size);
eSTATUS_t SpiDev_Transactions (SpiDev_t* pDev, SpiDevTransaction_t* pTransactions, uint32_t nTransactions);

// clang-format on

#endif /* PERIPHS_SPI_H */