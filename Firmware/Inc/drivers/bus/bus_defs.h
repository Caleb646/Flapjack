#ifndef DRIVERS_BUS_BUS_DEFS_H
#define DRIVERS_BUS_BUS_DEFS_H

#include <stdint.h>

#include "drivers/io/gpio_defs.h"

typedef uint8_t eBUS_DEV_ID_t; // can be eSPI_DEV_ID_t, eI2C_DEV_ID_t, etc.
enum {
    eBUS_ID_NULL = 0,

    eBUS_ID_SPI_1,
    eBUS_ID_SPI_2,
    eBUS_ID_SPI_3,
    eBUS_ID_SPI_4,
    eBUS_ID_SPI_5,

    eBUS_ID_I2C_1,
    eBUS_ID_I2C_2,
    eBUS_ID_I2C_3,
    eBUS_ID_I2C_4,
    eBUS_ID_I2C_5,
};

// BUS_ID_MAKE(SPI1) -> eBUS_DEV_ID_SPI1
#define BUS_ID_MAKE_EXPAND(DEVNAME) eBUS_ID_##DEVNAME
#define BUS_ID_MAKE(...)            BUS_ID_MAKE_EXPAND (__VA_ARGS__)

typedef uint8_t eBUS_TYPE_t;
enum { eBUS_TYPE_NULL = 0, eBUS_TYPE_SPI, eBUS_TYPE_I2C };

typedef struct {
    eBUS_DEV_ID_t busId;
    eBUS_TYPE_t busType;
    eGPIO_ID_t spiNssGpioId;
    uint8_t i2cAddress;
} BusDeviceCfg_t;

typedef struct SpiDevice_s SpiDevice_t;

typedef struct BusDeviceSPI_s {
    SpiDevice_t* pDev;
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