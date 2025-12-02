#ifndef CFG_IMU_H
#define CFG_IMU_H

#include <stdint.h>

#include "cfg/cfg.h"

#include "targets/target.h"

#include "cfg/bus/bus.h"


typedef struct {
    uint8_t devId;

    BusDeviceCfg_t busCfg;

    uint8_t extiGpioId;

    uint8_t alignment;
    uint8_t sampleRateHz;
} IMUCfg_t;

CFG_DECLARE_ARRAY (IMUCfg_t, IMUCfg, TARG_MAX_IMUS);

#define IMU_CFG_INIT(DEV_ID, ALIGNMENT, SAMPLE_RATE_HZ)                        \
    IMUCfg_GetMutable (DEV_ID_TO_CFG_ID (DEV_ID))->devId        = (DEV_ID);    \
    IMUCfg_GetMutable (DEV_ID_TO_CFG_ID (DEV_ID))->alignment    = (ALIGNMENT); \
    IMUCfg_GetMutable (DEV_ID_TO_CFG_ID (DEV_ID))->sampleRateHz = (SAMPLE_RATE_HZ)

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


#endif // CFG_IMU_H