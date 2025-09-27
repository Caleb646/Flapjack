#ifndef CONF_IDS_H
#define CONF_IDS_H

#include <stdint.h>

typedef uint8_t eGPIO_PINID_t;
enum {
    eGPIO_PINID_0  = 0U,
    eGPIO_PINID_1  = 1U,
    eGPIO_PINID_2  = 2U,
    eGPIO_PINID_3  = 3U,
    eGPIO_PINID_4  = 4U,
    eGPIO_PINID_5  = 5U,
    eGPIO_PINID_6  = 6U,
    eGPIO_PINID_7  = 7U,
    eGPIO_PINID_8  = 8U,
    eGPIO_PINID_9  = 9U,
    eGPIO_PINID_10 = 10U,
    eGPIO_PINID_11 = 11U,
    eGPIO_PINID_12 = 12U,
    eGPIO_PINID_13 = 13U,
    eGPIO_PINID_14 = 14U,
    eGPIO_PINID_15 = 15U,
    eGPIO_PINID_MAX
};

// #define GPIO_CHANNELID_NBITS    ((__builtin_ctz (eGPIO_PINID_MAX)))
#define GPIO_CHANNELID_NBITS (4U)

/* NOTE: because the port ids are not powers of two they
 cannot be or'ed together.
 Also the first port id starts at 1 so 0 can be used
 to indicate an invalid port. */
typedef uint8_t eGPIO_PORTID_t;
enum {
    eGPIO_PORTID_A = 1U * (eGPIO_PINID_MAX * 1U),
    eGPIO_PORTID_B = 1U * (eGPIO_PINID_MAX * 2U),
    eGPIO_PORTID_C = 1U * (eGPIO_PINID_MAX * 3U),
    eGPIO_PORTID_D = 1U * (eGPIO_PINID_MAX * 4U),
    eGPIO_PORTID_E = 1U * (eGPIO_PINID_MAX * 5U),
    eGPIO_PORTID_F = 1U * (eGPIO_PINID_MAX * 6U),
    eGPIO_PORTID_G = 1U * (eGPIO_PINID_MAX * 7U),
    eGPIO_PORTID_H = 1U * (eGPIO_PINID_MAX * 8U),
    eGPIO_PORTID_I = 1U * (eGPIO_PINID_MAX * 9U),
    eGPIO_PORTID_J = 1U * (eGPIO_PINID_MAX * 10U),
    eGPIO_PORTID_K = 1U * (eGPIO_PINID_MAX * 11U),
    eGPIO_PORTID_END__
};

#define eGPIO_PORTID_MAX ((eGPIO_PORTID_END__ / eGPIO_PINID_MAX))

// id = port id | pin id
typedef uint8_t eGPIO_ID_t;
enum {
    eGPIO_ID_NULL = 0U,
};

// #define GPIO_ID2PORTIDX(id) (((id) >> 15U) > 0U ? __builtin_ctz ((id) >> 15U) : 0U)
#define GPIO_ID2PORTIDX(id)     (((id) >> GPIO_CHANNELID_NBITS) - 1U)
#define GPIO_ID2PINIDX(id)      ((id) & (~(eGPIO_PINID_MAX - 1U)))
#define GPIO_ID_MAKE(port, pin) ((port) | (pin))
#define GPIO_ID_IS_GPIO(id) \
    ((id) != eGPIO_ID_NULL && (GPIO_ID2PORTIDX ((id)) < eGPIO_PORTID_MAX))

typedef uint8_t eDEVICE_ID_t;
enum {
    eDEVICE_ID_NULL = 0,
    eIMU_DEVICE_ID,
    eGPS_DEVICE_ID,
    eBARO_DEVICE_ID,
    eMAG_DEVICE_ID,
    eRF_RECEIVER_DEVICE_ID,
    eFLASH_DEVICE_ID,
    eCURRENT_SENSOR_DEVICE_ID,
    eSERIAL_DEBUG_DEVICE_ID,

    eSERVO_ID_BEGIN__,
    eSERVO_1_ID = eSERVO_ID_BEGIN__,
    eSERVO_2_ID,
    eSERVO_3_ID,
    eSERVO_4_ID,
    eSERVO_5_ID,
    eSERVO_6_ID,
    eSERVO_7_ID,
    eSERVO_8_ID,
    eSERVO_ID_END__,
    eSERVO_ID_MAX = eSERVO_ID_END__ - eSERVO_ID_BEGIN__,

    eMOTOR_ID_BEGIN__,
    eMOTOR_1_ID = eMOTOR_ID_BEGIN__,
    eMOTOR_2_ID,
    eMOTOR_ID_END__,
    eMOTOR_ID_MAX = eMOTOR_ID_END__ - eMOTOR_ID_BEGIN__,

    eDEVICE_ID_MAX
};

#define SERVO_ID2IDX(id) ((id) - eSERVO_ID_BEGIN__)
#define DEVICE_ID_IS_SERVO(id) \
    ((id) >= eSERVO_ID_BEGIN__ && (id) < eSERVO_ID_END__)

#define MOTOR_ID2IDX(id) ((id) - eMOTOR_ID_BEGIN__)
#define DEVICE_ID_IS_MOTOR(id) \
    ((id) >= eMOTOR_ID_BEGIN__ && (id) < eMOTOR_ID_END__)

typedef uint8_t eBUS_ID_t;
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
    eUART_5_BUS_ID,
    eUART_6_BUS_ID,
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

typedef uint8_t eTIMER_CHANNEL_ID_t;
enum {
    eTIMER_CHANNEL_1_ID = 0,
    eTIMER_CHANNEL_2_ID,
    eTIMER_CHANNEL_3_ID,
    eTIMER_CHANNEL_4_ID,
    eTIMER_CHANNEL_ID_MAX
};

#define TIMER_CHANNEL_ID_NBITS      2U
#define TIMER_CHANNEL_ID_MASK       (eTIMER_CHANNEL_ID_MAX - 1U)
#define TIMER_CHANNEL_HAL_ID_OFFSET 4U

typedef uint8_t eTIMER_DEV_ID_t;
enum {
    eTIMER_5_DEV_ID  = 1U * (eTIMER_CHANNEL_ID_MAX * 1U),
    eTIMER_8_DEV_ID  = 1U * (eTIMER_CHANNEL_ID_MAX * 2U),
    eTIMER_12_DEV_ID = 1U * (eTIMER_CHANNEL_ID_MAX * 3U),
    eTIMER_13_DEV_ID = 1U * (eTIMER_CHANNEL_ID_MAX * 4U),
    eTIMER_DEV_ID_MAX
};

// id = timer id | channel id
typedef uint8_t eTIMER_ID_t;
enum {
    eTIMER_ID_NULL = 0U,
};

#define eTIMER_ID_MAX            ((eTIMER_DEV_ID_MAX / eTIMER_CHANNEL_ID_MAX))
#define TIMER_ID2IDX(id)         (((id) >> TIMER_CHANNEL_ID_NBITS) - 1U)
#define TIMER_ID2CHANNEL_IDX(id) ((id) & TIMER_CHANNEL_ID_MASK)
#define TIMER_ID2HALCHANNEL(id) \
    ((TIM_CHANNEL_1) + (TIMER_ID2CHANNEL_IDX ((id)) * TIMER_CHANNEL_HAL_ID_OFFSET))
#define TIMER_ID_CLEAR_CHANNEL_BITS(id) ((id) & (~TIMER_CHANNEL_ID_MASK))
#define TIMER_ID_IS_TIMER(id) \
    ((((id) != eTIMER_ID_NULL) && (TIMER_ID2IDX ((id)) < eTIMER_ID_MAX)))
#define TIMER_ID_MAKE(timerId, channelId) ((timerId) | (channelId))

typedef uint8_t eEXTI_ID_t;
enum {
    eEXTI_ID_NULL = 0,
    eEXTI_0_ID,
    eEXTI_1_ID,
    eEXTI_2_ID,
    eEXTI_3_ID,
    eEXTI_4_ID,
    eEXTI_5_ID,
    eEXTI_6_ID,
    eEXTI_7_ID,
    eEXTI_8_ID,
    eEXTI_9_ID,
    eEXTI_10_ID,
    eEXTI_11_ID,
    eEXTI_12_ID,
    eEXTI_13_ID,
    eEXTI_14_ID,
    eEXTI_15_ID,
    eEXTI_ID_MAX
};

#define EXTI_ID_IS_EXTI(id) ((id) < eEXTI_ID_MAX)
#define EXTI_ID2IDX(id)     (id)

#endif // CONF_IDS_H