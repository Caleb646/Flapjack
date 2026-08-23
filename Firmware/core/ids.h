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


// id = port id | pin id
typedef uint8_t eGPIO_ID_t;
enum {
    eGPIO_ID_NULL = 0U,
};

#define GPIO_PIN_ID_NBITS             (4U)
#define GPIO_MAX_PINS                 (eGPIO_PINID_MAX)
#define GPIO_MAX_PORTS                ((eGPIO_PORTID_END__ / eGPIO_PINID_MAX))
#define GPIO_ID_TO_PORT_IDX(ID)       (((ID) >> GPIO_PIN_ID_NBITS) - 1U)
#define GPIO_ID_TO_PIN_IDX(ID)        ((ID) & (eGPIO_PINID_MAX - 1U))
#define GPIO_ID_MAKE(PORT_ID, PIN_ID) ((eGPIO_ID_t)(PORT_ID) | (eGPIO_ID_t)(PIN_ID))
#define GPIO_ID_VALID(ID)             ((ID) != eGPIO_ID_NULL && (GPIO_ID_TO_PORT_IDX ((ID)) < GPIO_MAX_PORTS))

typedef uint8_t eDEVICE_ID_t;
enum {
    eDEVICE_ID_NULL = 0U,
    eIMU_DEVICE_ID,
    eGPS_DEVICE_ID,
    eBARO_DEVICE_ID,
    eMAG_DEVICE_ID,
    eRF_RECEIVER_DEVICE_ID,
    eFLASH_DEVICE_ID,
    eCURRENT_SENSOR_DEVICE_ID,
    eSERIAL_LINK_DEVICE_ID,

    eDEVICE_ID_MAX
};

typedef uint8_t eBUS_ID_t;
enum {
    eNULL_BUS_ID = 0U,

    eI2C_BUS_ID_BEGIN__,
    eI2C_1_BUS_ID = eI2C_BUS_ID_BEGIN__,
    eI2C_2_BUS_ID,
    eI2C_BUS_ID_END__,

    eSPI_BUS_ID_BEGIN__,
    eSPI_1_BUS_ID = eSPI_BUS_ID_BEGIN__,
    eSPI_2_BUS_ID,
    eSPI_3_BUS_ID,
    eSPI_4_BUS_ID,
    eSPI_5_BUS_ID,
    eSPI_BUS_ID_END__,

    eUART_BUS_ID_BEGIN__,
    eUART_1_BUS_ID = eUART_BUS_ID_BEGIN__,
    eUART_2_BUS_ID,
    eUART_3_BUS_ID,
    eUART_4_BUS_ID,
    eUART_5_BUS_ID,
    eUART_6_BUS_ID,
    eUART_BUS_ID_END__,

    eBUS_ID_MAX
};

#define BUS_MAX_BUSES          (eBUS_ID_MAX - 1U)
#define SPI_MAX_BUSES          (eSPI_BUS_ID_END__ - eSPI_BUS_ID_BEGIN__)
#define UART_MAX_BUSES         (eUART_BUS_ID_END__ - eUART_BUS_ID_BEGIN__)
#define I2C_MAX_BUSES          (eI2C_BUS_ID_END__ - eI2C_BUS_ID_BEGIN__)

#define BUS_ID_IS_I2C(ID)      ((ID) >= eI2C_BUS_ID_BEGIN__ && (ID) < eI2C_BUS_ID_END__)
#define BUS_ID_IS_SPI(ID)      ((ID) >= eSPI_BUS_ID_BEGIN__ && (ID) < eSPI_BUS_ID_END__)
#define BUS_ID_IS_UART(ID)     ((ID) >= eUART_BUS_ID_BEGIN__ && (ID) < eUART_BUS_ID_END__)
#define SPI_BUS_ID_TO_IDX(ID)  ((ID) - eSPI_BUS_ID_BEGIN__)
#define UART_BUS_ID_TO_IDX(ID) ((ID) - eUART_BUS_ID_BEGIN__)
#define I2C_BUS_ID_TO_IDX(ID)  ((ID) - eI2C_BUS_ID_BEGIN__)

typedef uint8_t eTIMER_CHANNEL_ID_t;
enum {
    eTIMER_CHANNEL_1_ID = 0U,
    eTIMER_CHANNEL_2_ID,
    eTIMER_CHANNEL_3_ID,
    eTIMER_CHANNEL_4_ID,

    eTIMER_CHANNEL_ID_MAX
};

typedef uint8_t eTIMER_DEVICE_ID_t;
enum {
    eTIMER_5_DEVICE_ID  = 1U * (eTIMER_CHANNEL_ID_MAX * 1U),
    eTIMER_8_DEVICE_ID  = 1U * (eTIMER_CHANNEL_ID_MAX * 2U),
    eTIMER_12_DEVICE_ID = 1U * (eTIMER_CHANNEL_ID_MAX * 3U),
    eTIMER_13_DEVICE_ID = 1U * (eTIMER_CHANNEL_ID_MAX * 4U),

    eTIMER_DEVICE_ID_MAX
};

// id = timer id | channel id
typedef uint8_t eTIMER_ID_t;
enum {
    eTIMER_ID_NULL = 0U,
};

// clang-format off
#define TIMER_CHAN_ID_NBITS        2U
#define TIMER_CHAN_ID_MASK         (eTIMER_CHANNEL_ID_MAX - 1U)
#define TIMER_CHAN_HAL_ID_OFFSET   4U
#define TIMER_ID_CLR_CHAN_BITS(ID) ((ID) & (~TIMER_CHAN_ID_MASK))

#define TIMER_MAX_DEVICES          (eTIMER_DEVICE_ID_MAX / eTIMER_CHANNEL_ID_MAX)
#define TIMER_MAX_CHANNELS         eTIMER_CHANNEL_ID_MAX
#define TIMER_MAX_TIMERS           (TIMER_MAX_DEVICES * TIMER_MAX_CHANNELS)

#define TIMER_ID_TO_DEVICE_IDX(ID) (((ID) >> TIMER_CHAN_ID_NBITS) - 1U)
#define TIMER_ID_TO_CHAN_IDX(ID)   ((ID) & TIMER_CHAN_ID_MASK)
#define TIMER_ID_TO_IDX(ID)        (TIMER_ID_CLR_CHAN_BITS((ID)) - 1U)

#define TIMER_ID_TO_HAL_CHAN(ID) ((TIM_CHANNEL_1) + (TIMER_ID_TO_CHAN_IDX ((ID)) * TIMER_CHAN_HAL_ID_OFFSET))
#define TIMER_ID_VALID(ID) ((((ID) != eTIMER_ID_NULL) && (TIMER_ID_TO_IDX ((ID)) < TIMER_MAX_TIMERS)))
#define TIMER_ID_MAKE(TIMER_ID, CHAN_ID) ((eTIMER_ID_t)(TIMER_ID) | (eTIMER_ID_t)(CHAN_ID))

// clang-format on

typedef uint8_t eDSHOT_TYPE_t;
enum { eDSHOT_TYPE_150 = 0, eDSHOT_TYPE_300 = 1, eDSHOT_TYPE_600 = 2 };


#endif // CONF_IDS_H