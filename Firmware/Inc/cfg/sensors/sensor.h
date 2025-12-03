#ifndef CFG_SENSORS_SENSOR_H
#define CFG_SENSORS_SENSOR_H

#include <stdint.h>

#include "cfg/cfg.h"

#include "cfg/bus/bus.h"

#include "drivers/io/gpio_defs.h"

#include "targets/target.h"

typedef struct InertialCfg_s {
    uint8_t id;
    BusDeviceCfg_t busCfg;
    eGPIO_ID_t extiGpioId;
    uint8_t alignment;
    uint8_t sampleRateHz;
} InertialCfg_t;

CFG_DECLARE_ARRAY (InertialCfg_t, InertialCfg, TARG_MAX_IMUS * 3U);

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