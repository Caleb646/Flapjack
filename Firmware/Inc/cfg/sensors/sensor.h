#ifndef CFG_SENSORS_SENSOR_H
#define CFG_SENSORS_SENSOR_H

#include <stdint.h>

#include "cfg/cfg.h"

#include "drivers/bus/bus_defs.h"

#include "drivers/io/gpio_defs.h"

#include "drivers/sensors/inertial/inertial.h"

#include "targets/target.h"

#if !defined(TARG_MAX_ACCS) || !defined(TARG_MAX_GYROS) || !defined(TARG_MAX_MAGS)
#error "TARG_MAX_ACCS, TARG_MAX_GYROS, and TARG_MAX_MAGS must be defined in target.h"
#endif

#if TARG_MAX_ACCS < 1 || TARG_MAX_GYROS < 1 || TARG_MAX_MAGS < 1
#error "TARG_MAX_ACCS, TARG_MAX_GYROS, and TARG_MAX_MAGS must be at least 1"
#endif

typedef struct AccCfg_s {
    INER_TYPE_t type;
    BusDeviceCfg_t busCfg;
    eGPIO_ID_t extiGpioId;
    uint8_t alignment;
    uint16_t sampleRateHz;
} AccCfg_t;

typedef struct GyroCfg_s {
    INER_TYPE_t type;
    BusDeviceCfg_t busCfg;
    eGPIO_ID_t extiGpioId;
    uint8_t alignment;
    uint16_t sampleRateHz;
} GyroCfg_t;

typedef struct MagCfg_s {
    INER_TYPE_t type;
    BusDeviceCfg_t busCfg;
    eGPIO_ID_t extiGpioId;
    uint8_t alignment;
    uint16_t sampleRateHz;
} MagCfg_t;


CFG_DECLARE (AccCfg_t, AccCfg);
CFG_DECLARE (GyroCfg_t, GyroCfg);
CFG_DECLARE (MagCfg_t, MagCfg);

#define I_INERTIAL_CFG_GET(IDX)     InertialCfg_GetMutable (IDX)
#define I_INERTIAL_CFG_INIT(DEV_ID) I_INERTIAL_CFG_GET (DEV_ID_TO_CFG_ID (DEV_ID))->id = (DEV_ID)
#define I_INERTIAL_CFG_SET_SPI(DEV_ID, SPI_BUS_ID, NSS_GPIO_ID)                          \
    I_INERTIAL_CFG_GET (DEV_ID_TO_CFG_ID (DEV_ID))->busCfg.busType      = eBUS_TYPE_SPI; \
    I_INERTIAL_CFG_GET (DEV_ID_TO_CFG_ID (DEV_ID))->busCfg.busId        = (SPI_BUS_ID);  \
    I_INERTIAL_CFG_GET (DEV_ID_TO_CFG_ID (DEV_ID))->busCfg.spiNssGpioId = (NSS_GPIO_ID)

#define ACC_CFG_INIT(DEV_ID) I_INERTIAL_CFG_INIT (DEV_ID)
#define ACC_CFG_SET_SPI(DEV_ID, SPI_BUS_ID, NSS_GPIO_ID) \
    I_INERTIAL_CFG_SET_SPI (DEV_ID, SPI_BUS_ID, NSS_GPIO_ID)

#define IMU_CFG_INIT(DEV_ID, ALIGNMENT, SAMPLE_RATE_HZ)                         \
    I_INERTIAL_CFG_GET (DEV_ID_TO_CFG_ID (DEV_ID))->id           = (DEV_ID);    \
    I_INERTIAL_CFG_GET (DEV_ID_TO_CFG_ID (DEV_ID))->alignment    = (ALIGNMENT); \
    I_INERTIAL_CFG_GET (DEV_ID_TO_CFG_ID (DEV_ID))->sampleRateHz = (SAMPLE_RATE_HZ)

#define IMU_CFG_SET_SPI(DEV_ID, SPI_BUS_ID, NSS_GPIO_ID)                         \
    IMUCfg_GetMutable (DEV_ID_TO_CFG_ID (DEV_ID))->busType      = eBUS_TYPE_SPI; \
    IMUCfg_GetMutable (DEV_ID_TO_CFG_ID (DEV_ID))->busId        = (SPI_BUS_ID);  \
    IMUCfg_GetMutable (DEV_ID_TO_CFG_ID (DEV_ID))->spiNssGpioId = (NSS_GPIO_ID)

#define IMU_CFG_SET_I2C(DEV_ID, I2C_BUS_ID, I2C_ADDRESS)                       \
    IMUCfg_GetMutable (DEV_ID_TO_CFG_ID (DEV_ID))->busType    = eBUS_TYPE_I2C; \
    IMUCfg_GetMutable (DEV_ID_TO_CFG_ID (DEV_ID))->busId      = (I2C_BUS_ID);  \
    IMUCfg_GetMutable (DEV_ID_TO_CFG_ID (DEV_ID))->i2cAddress = (I2C_ADDRESS)

#define IMU_CFG_SET_EXTI(DEV_ID, EXTI_GPIO_ID) \
    IMUCfg_GetMutable (DEV_ID_TO_CFG_ID (DEV_ID))->extiGpioId = (EXTI_GPIO_ID)


#endif // CFG_SENSORS_SENSOR_H