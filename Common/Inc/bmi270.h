#ifndef BMI270_H
#define BMI270_H

#include "stdint.h"


/*! @name BMI2 register addresses */
#define BMI2_CHIP_ID_ADDR                             ((uint8_t)0x00)
#define BMI2_ERROR_ADDR                               ((uint8_t)0x02)
#define BMI2_STATUS_ADDR                              ((uint8_t)0x03)
#define BMI2_AUX_X_LSB_ADDR                           ((uint8_t)0x04)
#define BMI2_ACC_X_LSB_ADDR                           ((uint8_t)0x0C)
#define BMI2_GYR_X_LSB_ADDR                           ((uint8_t)0x12)
#define BMI2_SENSORTIME_ADDR                          ((uint8_t)0x18)
#define BMI2_EVENT_ADDR                               ((uint8_t)0x1B)
#define BMI2_INT_STATUS_0_ADDR                        ((uint8_t)0x1C)
#define BMI2_INT_STATUS_1_ADDR                        ((uint8_t)0x1D)
#define BMI2_SC_OUT_0_ADDR                            ((uint8_t)0x1E)
#define BMI2_SYNC_COMMAND_ADDR                        ((uint8_t)0x1E)
#define BMI2_GYR_CAS_GPIO0_ADDR                       ((uint8_t)0x1E)
#define BMI2_INTERNAL_STATUS_ADDR                     ((uint8_t)0x21)
#define BMI2_TEMPERATURE_0_ADDR                       ((uint8_t)0x22)
#define BMI2_TEMPERATURE_1_ADDR                       ((uint8_t)0x23)
#define BMI2_FIFO_LENGTH_0_ADDR                       ((uint8_t)0x24)
#define BMI2_FIFO_DATA_ADDR                           ((uint8_t)0x26)
#define BMI2_FEAT_PAGE_ADDR                           ((uint8_t)0x2F)
#define BMI2_FEATURES_REG_ADDR                        ((uint8_t)0x30)
#define BMI2_ACC_CONF_ADDR                            ((uint8_t)0x40)
#define BMI2_GYR_CONF_ADDR                            ((uint8_t)0x42)
#define BMI2_AUX_CONF_ADDR                            ((uint8_t)0x44)
#define BMI2_FIFO_DOWNS_ADDR                          ((uint8_t)0x45)
#define BMI2_FIFO_WTM_0_ADDR                          ((uint8_t)0x46)
#define BMI2_FIFO_WTM_1_ADDR                          ((uint8_t)0x47)
#define BMI2_FIFO_CONFIG_0_ADDR                       ((uint8_t)0x48)
#define BMI2_FIFO_CONFIG_1_ADDR                       ((uint8_t)0x49)
#define BMI2_SATURATION_ADDR                          ((uint8_t)0x4A)
#define BMI2_AUX_DEV_ID_ADDR                          ((uint8_t)0x4B)
#define BMI2_AUX_IF_CONF_ADDR                         ((uint8_t)0x4C)
#define BMI2_AUX_RD_ADDR                              ((uint8_t)0x4D)
#define BMI2_AUX_WR_ADDR                              ((uint8_t)0x4E)
#define BMI2_AUX_WR_DATA_ADDR                         ((uint8_t)0x4F)
#define BMI2_ERR_REG_MSK_ADDR                         ((uint8_t)0x52)
#define BMI2_INT1_IO_CTRL_ADDR                        ((uint8_t)0x53)
#define BMI2_INT2_IO_CTRL_ADDR                        ((uint8_t)0x54)
#define BMI2_INT_LATCH_ADDR                           ((uint8_t)0x55)
#define BMI2_INT1_MAP_FEAT_ADDR                       ((uint8_t)0x56)
#define BMI2_INT2_MAP_FEAT_ADDR                       ((uint8_t)0x57)
#define BMI2_INT_MAP_DATA_ADDR                        ((uint8_t)0x58)
#define BMI2_INIT_CTRL_ADDR                           ((uint8_t)0x59)
#define BMI2_INIT_ADDR_0                              ((uint8_t)0x5B)
#define BMI2_INIT_ADDR_1                              ((uint8_t)0x5C)
#define BMI2_INIT_DATA_ADDR                           ((uint8_t)0x5E)
#define BMI2_INTERNAL_ERR_ADDR                        ((uint8_t)0x5F)
#define BMI2_AUX_IF_TRIM                              ((uint8_t)0x68)
#define BMI2_GYR_CRT_CONF_ADDR                        ((uint8_t)0x69)
#define BMI2_NVM_CONF_ADDR                            ((uint8_t)0x6A)
#define BMI2_IF_CONF_ADDR                             ((uint8_t)0x6B)
#define BMI2_DRV_STR_ADDR                             ((uint8_t)0x6C)
#define BMI2_ACC_SELF_TEST_ADDR                       ((uint8_t)0x6D)
#define BMI2_GYR_SELF_TEST_AXES_ADDR                  ((uint8_t)0x6E)
#define BMI2_SELF_TEST_MEMS_ADDR                      ((uint8_t)0x6F)
#define BMI2_NV_CONF_ADDR                             ((uint8_t)0x70)
#define BMI2_ACC_OFF_COMP_0_ADDR                      ((uint8_t)0x71)
#define BMI2_GYR_OFF_COMP_3_ADDR                      ((uint8_t)0x74)
#define BMI2_GYR_OFF_COMP_6_ADDR                      ((uint8_t)0x77)
#define BMI2_GYR_USR_GAIN_0_ADDR                      ((uint8_t)0x78)
#define BMI2_PWR_CONF_ADDR                            ((uint8_t)0x7C)
#define BMI2_PWR_CTRL_ADDR                            ((uint8_t)0x7D)
#define BMI2_CMD_REG_ADDR                             ((uint8_t)0x7E)

/*! @name BMI2 I2C address */
#define BMI2_I2C_PRIM_ADDR                            ((uint8_t)0x68)
#define BMI2_I2C_SEC_ADDR                             ((uint8_t)0x69)

/*! @name BMI2 Commands */
#define BMI2_G_TRIGGER_CMD                            ((uint8_t)0x02)
#define BMI2_USR_GAIN_CMD                             ((uint8_t)0x03)
#define BMI2_NVM_PROG_CMD                             ((uint8_t)0xA0)
#define BMI2_SOFT_RESET_CMD                           ((uint8_t)0xB6)
#define BMI2_FIFO_FLUSH_CMD                           ((uint8_t)0xB0)

/*! @name BMI2 sensor data bytes */

#define BMI2_AUX_NUM_BYTES                            ((uint8_t)8)
#define BMI2_ACC_NUM_BYTES                            ((uint8_t)6)
#define BMI2_GYR_NUM_BYTES                            ((uint8_t)6)
#define BMI2_STATUS_INDEX                             ((uint8_t)0)
#define BMI2_AUX_START_INDEX                          ((uint8_t)1)
#define BMI2_ACC_START_INDEX                          ((uint8_t)9)
#define BMI2_GYR_START_INDEX                          ((uint8_t)15)
#define BMI2_ACC_GYR_AUX_SENSORTIME_NUM_BYTES         ((uint8_t)24)
#define BMI2_CRT_CONFIG_FILE_SIZE                     ((uint16_t)2048)
#define BMI2_FEAT_SIZE_IN_BYTES                       ((uint8_t)16)
#define BMI2_ACC_CONFIG_LENGTH                        ((uint8_t)2)

/*! @name BMI2 configuration load status */
#define BMI2_CONFIG_LOAD_SUCCESS                      ((uint8_t)1)
#define BMI2_CONFIG_LOAD_STATUS_MASK                  ((uint8_t)0x0F)

#define BMI2_INT_STATUS_ERROR_BIT   (1 << 1)
#define BMI2_INT_STATUS_AUX_RDY_BIT (1 << 5)
#define BMI2_INT_STATUS_GYR_RDY_BIT (1 << 6)
#define BMI2_INT_STATUS_ACC_RDY_BIT (1 << 7)

#define BMI2_CHIP_ID 0x24

#endif // BMI270_H
