#ifndef DRIVERS_BUS_SPI_DEFS_H
#define DRIVERS_BUS_SPI_DEFS_H

#include <stdint.h>

typedef uint8_t eSPI_DEV_ID_t;
enum {
    eSPI_DEV_ID_NULL = 0,
    eSPI_DEV_ID_1,
    eSPI_DEV_ID_2,
    eSPI_DEV_ID_3,
    eSPI_DEV_ID_4,
    eSPI_DEV_ID_5,
    eSPI_DEV_ID_6,
};

#define SPI_DEV_ID_TO_INDEX(DEV_ID) ((DEV_ID) - 1U)


struct SPI_Typedef;
typedef struct SPIDevice_s {
    eSPI_DEV_ID_t devId;
    struct SPI_Typedef* pInstance;
} SPIDevice_t;


#endif // DRIVERS_BUS_SPI_DEFS_H