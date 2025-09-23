#ifndef CONF_IDS_H
#define CONF_IDS_H

#include <stdint.h>

typedef uint32_t eGPIO_ID_t;
enum {
    eNULL_GPIO_ID = 0,

    eGPIO_A_0_ID  = (1U << 15U) | 0U,
    eGPIO_A_1_ID  = (1U << 15U) | 1U,
    eGPIO_A_2_ID  = (1U << 15U) | 2U,
    eGPIO_A_3_ID  = (1U << 15U) | 3U,
    eGPIO_A_4_ID  = (1U << 15U) | 4U,
    eGPIO_A_5_ID  = (1U << 15U) | 5U,
    eGPIO_A_6_ID  = (1U << 15U) | 6U,
    eGPIO_A_7_ID  = (1U << 15U) | 7U,
    eGPIO_A_8_ID  = (1U << 15U) | 8U,
    eGPIO_A_9_ID  = (1U << 15U) | 9U,
    eGPIO_A_10_ID = (1U << 15U) | 10U,
    eGPIO_A_11_ID = (1U << 15U) | 11U,
    eGPIO_A_12_ID = (1U << 15U) | 12U,
    eGPIO_A_13_ID = (1U << 15U) | 13U,
    eGPIO_A_14_ID = (1U << 15U) | 14U,
    eGPIO_A_15_ID = (1U << 15U) | 15U,

    eGPIO_B_0_ID  = (1U << 16U) | 0U,
    eGPIO_B_1_ID  = (1U << 16U) | 1U,
    eGPIO_B_2_ID  = (1U << 16U) | 2U,
    eGPIO_B_3_ID  = (1U << 16U) | 3U,
    eGPIO_B_4_ID  = (1U << 16U) | 4U,
    eGPIO_B_5_ID  = (1U << 16U) | 5U,
    eGPIO_B_6_ID  = (1U << 16U) | 6U,
    eGPIO_B_7_ID  = (1U << 16U) | 7U,
    eGPIO_B_8_ID  = (1U << 16U) | 8U,
    eGPIO_B_9_ID  = (1U << 16U) | 9U,
    eGPIO_B_10_ID = (1U << 16U) | 10U,
    eGPIO_B_11_ID = (1U << 16U) | 11U,
    eGPIO_B_12_ID = (1U << 16U) | 12U,
    eGPIO_B_13_ID = (1U << 16U) | 13U,
    eGPIO_B_14_ID = (1U << 16U) | 14U,
    eGPIO_B_15_ID = (1U << 16U) | 15U,

    eGPIO_C_0_ID  = (1U << 17U) | 0U,
    eGPIO_C_1_ID  = (1U << 17U) | 1U,
    eGPIO_C_2_ID  = (1U << 17U) | 2U,
    eGPIO_C_3_ID  = (1U << 17U) | 3U,
    eGPIO_C_4_ID  = (1U << 17U) | 4U,
    eGPIO_C_5_ID  = (1U << 17U) | 5U,
    eGPIO_C_6_ID  = (1U << 17U) | 6U,
    eGPIO_C_7_ID  = (1U << 17U) | 7U,
    eGPIO_C_8_ID  = (1U << 17U) | 8U,
    eGPIO_C_9_ID  = (1U << 17U) | 9U,
    eGPIO_C_10_ID = (1U << 17U) | 10U,
    eGPIO_C_11_ID = (1U << 17U) | 11U,
    eGPIO_C_12_ID = (1U << 17U) | 12U,

    eGPIO_D_0_ID  = (1U << 18U) | 0U,
    eGPIO_D_1_ID  = (1U << 18U) | 1U,
    eGPIO_D_2_ID  = (1U << 18U) | 2U,
    eGPIO_D_3_ID  = (1U << 18U) | 3U,
    eGPIO_D_4_ID  = (1U << 18U) | 4U,
    eGPIO_D_5_ID  = (1U << 18U) | 5U,
    eGPIO_D_6_ID  = (1U << 18U) | 6U,
    eGPIO_D_7_ID  = (1U << 18U) | 7U,
    eGPIO_D_8_ID  = (1U << 18U) | 8U,
    eGPIO_D_9_ID  = (1U << 18U) | 9U,
    eGPIO_D_10_ID = (1U << 18U) | 10U,
    eGPIO_D_11_ID = (1U << 18U) | 11U,
    eGPIO_D_12_ID = (1U << 18U) | 12U,
    eGPIO_D_13_ID = (1U << 18U) | 13U,
    eGPIO_D_14_ID = (1U << 18U) | 14U,
    eGPIO_D_15_ID = (1U << 18U) | 15U,

    eGPIO_E_0_ID = (1U << 19U) | 0U,

    eGPIO_F_0_ID  = (1U << 20U) | 0U,
    eGPIO_F_1_ID  = (1U << 20U) | 1U,
    eGPIO_F_2_ID  = (1U << 20U) | 2U,
    eGPIO_F_3_ID  = (1U << 20U) | 3U,
    eGPIO_F_4_ID  = (1U << 20U) | 4U,
    eGPIO_F_5_ID  = (1U << 20U) | 5U,
    eGPIO_F_6_ID  = (1U << 20U) | 6U,
    eGPIO_F_7_ID  = (1U << 20U) | 7U,
    eGPIO_F_8_ID  = (1U << 20U) | 8U,
    eGPIO_F_9_ID  = (1U << 20U) | 9U,
    eGPIO_F_10_ID = (1U << 20U) | 10U,
    eGPIO_F_11_ID = (1U << 20U) | 11U,
    eGPIO_F_12_ID = (1U << 20U) | 12U,
    eGPIO_F_13_ID = (1U << 20U) | 13U,

    eGPIO_G_0_ID = (1U << 21U) | 0U,
    eGPIO_H_0_ID = (1U << 22U) | 0U,

    eGPIO_ID_MAX
};

#define GPIO_ID2PORTIDX(id) ((id) >> 15U)
#define GPIO_ID2PINIDX(id)  ((id) & 0xFFU)

typedef uint16_t eDEVICE_ID_t;
enum {
    eNULL_DEVICE_ID = 0,
    eIMU_DEVICE_ID,
    eGPS_DEVICE_ID,
    eBARO_DEVICE_ID,
    eMAG_DEVICE_ID,
    eRF_RECEIVER_DEVICE_ID,
    eFLASH_DEVICE_ID,

    eSERVO_ID_BEGIN__,
    eLEFT_SERVO_1_ID = eSERVO_ID_BEGIN__,
    eLEFT_SERVO_2_ID,
    eLEFT_SERVO_3_ID,
    eLEFT_SERVO_4_ID,

    eRIGHT_SERVO_1_ID,
    eRIGHT_SERVO_2_ID,
    eRIGHT_SERVO_3_ID,
    eRIGHT_SERVO_4_ID,
    eSERVO_ID_END__,
    eSERVO_ID_MAX = eSERVO_ID_END__ - eSERVO_ID_BEGIN__,

    eMOTOR_ID_BEGIN__,
    eLEFT_MOTOR_ID = eMOTOR_ID_BEGIN__,
    eRIGHT_MOTOR_ID,
    eMOTOR_ID_END__,
    eMOTOR_ID_MAX = eMOTOR_ID_END__ - eMOTOR_ID_BEGIN__,

    eDEVICE_ID_MAX
};

#define SERVO_ID2IDX(id) ((id) - eSERVO_ID_BEGIN__)
#define MOTOR_ID2IDX(id) ((id) - eMOTOR_ID_BEGIN__)
#define DEVICE_ID_IS_SERVO(id) \
    ((id) >= eSERVO_ID_BEGIN__ && (id) < eSERVO_ID_END__)
#define DEVICE_ID_IS_MOTOR(id) \
    ((id) >= eMOTOR_ID_BEGIN__ && (id) < eMOTOR_ID_END__)

typedef uint16_t eBUS_ID_t;
enum {
    eNULL_BUS_ID = 0,

    eI2C_BUS_ID_BEGIN__,
    eI2C_1_BUS_ID = eI2C_BUS_ID_BEGIN__,
    eI2C_2_BUS_ID,
    eI2C_BUS_ID_END__,
    eI2C_BUS_ID_MAX = eI2C_BUS_ID_END__ - eI2C_BUS_ID_BEGIN__,

    eSPI_BUS_ID_BEGIN__,
    eSPI_1_BUS_ID = eSPI_BUS_ID_BEGIN__,
    eSPI_2_BUS_ID,
    eSPI_3_BUS_ID,
    eSPI_4_BUS_ID,
    eSPI_5_BUS_ID,
    eSPI_BUS_ID_END__,
    eSPI_BUS_ID_MAX = eSPI_BUS_ID_END__ - eSPI_BUS_ID_BEGIN__,

    eUART_BUS_ID_BEGIN__,
    eUART_1_BUS_ID = eUART_BUS_ID_BEGIN__,
    eUART_2_BUS_ID,
    eUART_3_BUS_ID,
    eUART_4_BUS_ID,
    eUART_BUS_ID_END__,
    eUART_BUS_ID_MAX = eUART_BUS_ID_END__ - eUART_BUS_ID_BEGIN__,

    eBUS_ID_MAX
};

#define BUS_ID_IS_I2C(id) \
    ((id) >= eI2C_BUS_ID_BEGIN__ && (id) < eI2C_BUS_ID_END__)
#define BUS_ID_IS_SPI(id) \
    ((id) >= eSPI_BUS_ID_BEGIN__ && (id) < eSPI_BUS_ID_END__)
#define BUS_ID_IS_UART(id) \
    ((id) >= eUART_BUS_ID_BEGIN__ && (id) < eUART_BUS_ID_END__)
#define SPI_BUS_ID2IDX(id)  ((id) - eSPI_BUS_ID_BEGIN__)
#define UART_BUS_ID2IDX(id) ((id) - eUART_BUS_ID_BEGIN__)
#define I2C_BUS_ID2IDX(id)  ((id) - eI2C_BUS_ID_BEGIN__)

typedef uint32_t eTIMER_ID_t;
enum {
    eTIMER_5_CH1_ID = 0 | 0,
    eTIMER_5_CH2_ID = 1 | 1,
    eTIMER_5_CH3_ID = 2 | 2,
    eTIMER_5_CH4_ID = 3 | 3,

    eTIMER_8_CH1_ID = 4 | 0,
    eTIMER_8_CH2_ID = 5 | 1,
    eTIMER_8_CH3_ID = 6 | 2,
    eTIMER_8_CH4_ID = 7 | 3,

    eTIMER_12_CH1_ID = 8 | 0,
    eTIMER_12_CH2_ID = 9 | 1,

    eTIMER_13_CH1_ID = 12 | 0,
    eTIMER_13_CH2_ID = 13 | 1,

    eTIMER_MAX_ID
};

#define TIMER_ID_CHANNEL_MASK           0b11U
#define TIMER_ID2CHANNEL(id)            ((TIM_CHANNEL_1) + ((id) & 0b11U) * 4U)
#define TIMER_ID2CHANNEL_IDX(id)        ((id) & TIMER_ID_CHANNEL_MASK)
#define TIMER_ID_CLEAR_CHANNEL_BITS(id) ((id) & ~TIMER_ID_CHANNEL_MASK)
#define TIMER_ID2IDX(id)                (((id) & ~TIMER_ID_CHANNEL_MASK) >> 2U)

#endif // CONF_IDS_H