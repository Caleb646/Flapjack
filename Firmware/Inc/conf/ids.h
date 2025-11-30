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

    eMOTOR_ID_BEGIN__,
    eMOTOR_1_ID = eMOTOR_ID_BEGIN__,
    eMOTOR_2_ID,
    eMOTOR_ID_END__,

    eDEVICE_ID_MAX
};

#define SERVO_MAX_SERVOS       (eSERVO_ID_END__ - eSERVO_ID_BEGIN__)
#define MOTOR_MAX_MOTORS       (eMOTOR_ID_END__ - eMOTOR_ID_BEGIN__)
#define DEVICE_MAX_DEVICES     (eDEVICE_ID_MAX - 1U)
#define SERVO_ID_TO_IDX(id)    ((id) - eSERVO_ID_BEGIN__)
#define DEVICE_ID_IS_SERVO(id) ((id) >= eSERVO_ID_BEGIN__ && (id) < eSERVO_ID_END__)
#define MOTOR_ID_TO_IDX(id)    ((id) - eMOTOR_ID_BEGIN__)
#define DEVICE_ID_IS_MOTOR(id) ((id) >= eMOTOR_ID_BEGIN__ && (id) < eMOTOR_ID_END__)

typedef uint32_t eDEVICE_FLAGS_t;
enum {
    eDEVICE_FLAG_NULL                = 0,
    eDEVICE_FLAG_USE_DMA_FOR_WRITES  = (1 << 1),
    eDEVICE_FLAG_USE_DMA_FOR_READS   = (1 << 2),
    eDEVICE_FLAG_USE_INTS_FOR_WRITES = (1 << 3),
    eDEVICE_FLAG_USE_INTS_FOR_READS  = (1 << 4),
};

#define DEVICE_FLAG_IS_SET(FLAGS, FLAG) (((FLAGS) & (FLAG)) != 0)


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

#define BUS_MAX_BUSES           (eBUS_ID_MAX - 1U)
#define SPI_MAX_BUSES           (eSPI_BUS_ID_END__ - eSPI_BUS_ID_BEGIN__)
#define UART_MAX_BUSES          (eUART_BUS_ID_END__ - eUART_BUS_ID_BEGIN__)
#define I2C_MAX_BUSES           (eI2C_BUS_ID_END__ - eI2C_BUS_ID_BEGIN__)

#define BUS_ID_IS_I2C(ID)       ((ID) >= eI2C_BUS_ID_BEGIN__ && (ID) < eI2C_BUS_ID_END__)
#define BUS_ID_IS_SPI(ID)       ((ID) >= eSPI_BUS_ID_BEGIN__ && (ID) < eSPI_BUS_ID_END__)
#define BUS_ID_IS_UART(ID)      ((ID) >= eUART_BUS_ID_BEGIN__ && (ID) < eUART_BUS_ID_END__)
#define SPI_BUS_ID_TO_IDX(ID)   ((ID) - eSPI_BUS_ID_BEGIN__)
#define UART_BUS_ID_TO_IDX(ID)  ((ID) - eUART_BUS_ID_BEGIN__)
#define I2C_BUS_ID_TO_IDX(ID)   ((ID) - eI2C_BUS_ID_BEGIN__)

#define SPI_MAX_DEVICES_PER_BUS 3U
typedef uint8_t eSPI_SPEED_t;
enum {
    eSPI_SPEED_NULL = 0U,
    eSPI_SPEED_125KHZ,
    eSPI_SPEED_250KHZ,
    eSPI_SPEED_500KHZ,
    eSPI_SPEED_1MHZ,
    eSPI_SPEED_2MHZ,
    eSPI_SPEED_4MHZ,
    eSPI_SPEED_8MHZ,
    eSPI_SPEED_16MHZ,
    eSPI_SPEED_32MHZ,
    eSPI_SPEED_64MHZ,
    eSPI_SPEED_128MHZ,
    eSPI_SPEED_256MHZ,
    eSPI_SPEED_MAX
};

typedef uint8_t eUART_BAUDRATE_t;
enum {
    eUART_BAUDRATE_NULL = 0U,
    eUART_BAUDRATE_9600,
    eUART_BAUDRATE_19200,
    eUART_BAUDRATE_38400,
    eUART_BAUDRATE_57600,
    eUART_BAUDRATE_115200,
    eUART_BAUDRATE_230400,
    eUART_BAUDRATE_460800,
    eUART_BAUDRATE_921600,
    eUART_BAUDRATE_1000000,
    eUART_BAUDRATE_MAX
};

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

typedef uint8_t eEXTI_ID_t;
enum {
    eEXTI_ID_NULL = 0U,
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

#define EXTI_ID_VALID(ID)  ((ID) < eEXTI_ID_MAX)
#define EXTI_ID_TO_IDX(ID) (ID)

typedef uint8_t eDSHOT_SPEED_t;
enum { eDSHOT_SPEED_NULL, eDSHOT_SPEED_150, eDSHOT_SPEED_300, eDSHOT_SPEED_600, eDSHOT_SPEED_MAX };

typedef uint8_t eDSHOT_TYPE_t;
enum {
    eDSHOT_TYPE_NULL = 0U,
    eDSHOT_TYPE_DMA_TO_TIMER,
    eDSHOT_TYPE_TIMER_ONLY,
    eDSHOT_TYPE_BB_ONLY,
    eDSHOT_TYPE_MAX
};

#define DSHOT_SPEED_VALID(SPEED) ((SPEED) > eDSHOT_SPEED_NULL && (SPEED) < eDSHOT_SPEED_MAX)

typedef uint8_t ePID_TYPE_t;
enum { ePID_TYPE_NULL = 0U, ePID_TYPE_ANGLE, ePID_TYPE_RATE, ePID_TYPE_MAX };

#define PID_TYPE_VALID(TYPE) ((TYPE) > ePID_TYPE_NULL && (TYPE) < ePID_TYPE_MAX)

typedef uint8_t eACTUATOR_PROTOCOL_t;
enum {
    eACTUATOR_PROTOCOL_NULL = 0U,
    eACTUATOR_PROTOCOL_PWM,
    eACTUATOR_PROTOCOL_DSHOT,
    eACTUATOR_PROTOCOL_MAX
};

#define ACTUATOR_PROTOCOL_VALID(TYPE) \
    ((TYPE) > eACTUATOR_PROTOCOL_NULL && (TYPE) < eACTUATOR_PROTOCOL_MAX)

#endif // CONF_IDS_H