#ifndef DRIVERS_BUS_BUS_DEFS_H
#define DRIVERS_BUS_BUS_DEFS_H

#include <stdint.h>

#include "drivers/bus/spi_defs.h"

#include "drivers/io/gpio_defs.h"

typedef uint8_t eBUS_DEV_ID_t; // can be eSPI_DEV_ID_t, eI2C_DEV_ID_t, etc.

typedef uint8_t eBUS_TYPE_t;
enum { eBUS_TYPE_NULL = 0, eBUS_TYPE_SPI, eBUS_TYPE_I2C };

typedef struct {
    eBUS_DEV_ID_t busId;
    eBUS_TYPE_t busType;

    eGPIO_ID_t spiSckGpioId;
    eGPIO_ID_t spiMisoGpioId;
    eGPIO_ID_t spiMosiGpioId;
    eGPIO_ID_t spiNssGpioId;

    eGPIO_ID_t i2cSclGpioId;
    eGPIO_ID_t i2cSdaGpioId;
    uint8_t i2cAddress;
} BusDeviceCfg_t;

typedef struct BusDeviceSPI_s {
    SPIDevice_t* pDev;
    GPIO_t* pNSS;
} BusDeviceSPI_t;

typedef struct BusDeviceI2C_s {
    // I2CDevice_t* pDev;
    // GPIO_t* pSCL;
    // GPIO_t* pSDA;
    int dummy;
} BusDeviceI2C_t;

typedef struct BusDevice_s {
    eBUS_DEV_ID_t devId;
    eBUS_TYPE_t busType;
    union {
        BusDeviceSPI_t spi;
        BusDeviceI2C_t i2c;
    };
} BusDevice_t;


#endif // DRIVERS_BUS_BUS_DEFS_H