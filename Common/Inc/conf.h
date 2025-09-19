#ifndef CONF_H
#define CONF_H
#include "hal.h"

// #define USE_SERVOS_ONLY

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

    eMOTOR_ID_BEGIN__,
    eLEFT_MOTOR_ID = eMOTOR_ID_BEGIN__,
    eRIGHT_MOTOR_ID,
    eMOTOR_ID_END__
};

#define eSERVO_ID_MAX (eSERVO_ID_END__ - eSERVO_ID_BEGIN__)
#define eMOTOR_ID_MAX (eMOTOR_ID_END__ - eMOTOR_ID_BEGIN__)
#define DEVICE_ID_IS_SERVO(id) \
    ((id) >= eSERVO_ID_BEGIN__ && (id) < eSERVO_ID_END__)
#define DEVICE_ID_IS_MOTOR(id) \
    ((id) >= eMOTOR_ID_BEGIN__ && (id) < eMOTOR_ID_END__)

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

#define TIMER_ID_CHANNEL_MASK 0b11U
#define TIMER_ID2CHANNEL(id)  ((TIM_CHANNEL_1) + ((id) & 0b11U) * 4U)

typedef uint8_t eUART_BUS_ID_t;
enum {
    eUART_1_BUS_ID = 0,
    eUART_2_BUS_ID,
    eUART_3_BUS_ID,
    eUART_4_BUS_ID,
    eUART_BUS_ID_MAX
};

typedef uint8_t eSPI_BUS_ID_t;
enum {
    eSPI_1_BUS_ID = 0,
    eSPI_2_BUS_ID,
    eSPI_3_BUS_ID,
    eSPI_4_BUS_ID,
    eSPI_5_BUS_ID,
    eSPI_BUS_ID_MAX
};

#define IMU_INT_EXTI_IRQn                EXTI9_5_IRQn
#define IMU_INT_GPIO_Pin                 GPIO_PIN_5
#define IMU_SPI_BUS_ID                   eSPI_1_BUS_ID

#define MAG_INT_EXTI_IRQn                EXTI3_IRQn
#define MAG_INT_GPIO_Pin                 GPIO_PIN_3
#define MAG_SPI_BUS_ID                   eSPI_5_BUS_ID

#define BARO_INT_EXTI_IRQn               EXTI9_5_IRQn
#define BARO_INT_GPIO_Pin                GPIO_PIN_6
#define BARO_SPI_BUS_ID                  eSPI_5_BUS_ID

#define FLASH_SPI_BUS_ID                 eSPI_3_BUS_ID

#define DEBUG_UART_BUS_ID                eUART_1_BUS_ID

#define GPS_UART_BUS_ID                  eUART_2_BUS_ID

#define RF_RECEIVER_UART_BUS_ID          eUART_3_BUS_ID

#define LEFT_SERVO_1_PWM_TIMER           eTIMER_5_ID
#define LEFT_SERVO_1_PWM_CHANNEL         TIM_CHANNEL_1
#define LEFT_SERVO_1_PWM_FREQUENCY       50U // 50 Hz

#define LEFT_SERVO_2_PWM_TIMER           eTIMER_5_ID
#define LEFT_SERVO_2_PWM_CHANNEL         TIM_CHANNEL_2
#define LEFT_SERVO_2_PWM_FREQUENCY       50U // 50 Hz

#define LEFT_SERVO_3_PWM_TIMER           eTIMER_5_ID
#define LEFT_SERVO_3_PWM_CHANNEL         TIM_CHANNEL_3
#define LEFT_SERVO_3_PWM_FREQUENCY       50U // 50 Hz

#define LEFT_SERVO_4_PWM_TIMER           eTIMER_5_ID
#define LEFT_SERVO_4_PWM_CHANNEL         TIM_CHANNEL_4
#define LEFT_SERVO_4_PWM_FREQUENCY       50U // 50 Hz

#define LEFT_MOTOR_PWM_TIMER             eTIMER_12_ID
#define LEFT_MOTOR_PWM_CHANNEL           TIM_CHANNEL_1
// TODO TIM12 doesnt support DMA request
// #define LEFT_MOTOR_DMA_STREAM            DMA1_Stream0
// #define LEFT_MOTOR_DMA_REQUEST           DMA_REQUEST_TIM16_CH1

#define RIGHT_SERVO_1_PWM_TIMER          eTIMER_8_ID
#define RIGHT_SERVO_1_PWM_CHANNEL        TIM_CHANNEL_1
#define RIGHT_SERVO_1_PWM_FREQUENCY      50U // 50 Hz

#define RIGHT_SERVO_2_PWM_TIMER          eTIMER_8_ID
#define RIGHT_SERVO_2_PWM_CHANNEL        TIM_CHANNEL_2
#define RIGHT_SERVO_2_PWM_FREQUENCY      50U // 50 Hz

#define RIGHT_SERVO_3_PWM_TIMER          eTIMER_8_ID
#define RIGHT_SERVO_3_PWM_CHANNEL        TIM_CHANNEL_3
#define RIGHT_SERVO_3_PWM_FREQUENCY      50U // 50 Hz

#define RIGHT_SERVO_4_PWM_TIMER          eTIMER_8_ID
#define RIGHT_SERVO_4_PWM_CHANNEL        TIM_CHANNEL_4
#define RIGHT_SERVO_4_PWM_FREQUENCY      50U // 50 Hz

#define RIGHT_MOTOR_PWM_TIMER            eTIMER_12_ID
#define RIGHT_MOTOR_PWM_CHANNEL          TIM_CHANNEL_2
// TODO TIM12 doesnt support DMA request
// #define RIGHT_MOTOR_DMA_STREAM            DMA1_Stream0
// #define RIGHT_MOTOR_DMA_REQUEST           DMA_REQUEST_TIM16_CH1


#define MS_PER_LOG_DATA_UPDATE           250U // 250 ms data update interval

#define HZ_SENSOR_UPDATE_RATE            200U // 200 Hz sensor update rate
#define SENSOR_UPDATE_MODE_IS_INTERRUPT  0U

#define PRIMARY_LOGGER_ROLE              CM4_CPUID
#define LOGGER_SHOULD_BLOCK_ON_OVERWRITE 1U

#define PID_MIN_VALUE                    0.0F
#define PID_MAX_VALUE                    5.0F
#define PID_STARTING_ROLL_P              0.2F
#define PID_STARTING_ROLL_I              0.3F
#define PID_STARTING_ROLL_D              0.05F
#define PID_STARTING_PITCH_P             0.2F
#define PID_STARTING_PITCH_I             0.3F
#define PID_STARTING_PITCH_D             0.05F
#define PID_STARTING_YAW_P               0.3F
#define PID_STARTING_YAW_I               0.05F
#define PID_STARTING_YAW_D               0.00015F
#define PID_STARTING_INTEGRAL_LIMIT      25.0F

#define MOTOR_MAX_THROTTLE               0.40F // 40% throttle
#define MOTOR_STARTUP_THROTTLE           0.25F // 25% throttle
#define MOTOR_MIN_THROTTLE               0.20F // 20% throttle

/*
 * PID mixing magnitudes for the Motors. They determine how much each PID axis contributes to the motor throttle.
 * For example (PID values are between -1 to 1) if a PID pitch value of 0.5 is received the motor should
 * increase its throttle to maintain the current altitude while allowing the drone to pitch up.
 */
#define MOTOR_PID_ROLL_MIX               0.0F
#define MOTOR_PID_PITCH_MIX              1.0F
#define MOTOR_PID_YAW_MIX                0.0F

/*
 * PID mixing directions for the Motors. They determine the direction of each PID axis contribution to the motor throttle.
 * For example (PID values are between -1 to 1) if a PID roll value of 0.5
 * is received the left motor should increase its throttle and the right motor should decrease its throttle.
 *
 * NOTE: A PID pitch value should always increase the throttle of both motors regardless of the sign of the PID pitch value.
 */
#define LEFT_MOTOR_PID_ROLL_MIX_DIR      1.0F
#define LEFT_MOTOR_PID_PITCH_MIX_DIR     1.0F
#define LEFT_MOTOR_PID_YAW_MIX_DIR       1.0F

#define RIGHT_MOTOR_PID_ROLL_MIX_DIR     1.0F
#define RIGHT_MOTOR_PID_PITCH_MIX_DIR    1.0F
#define RIGHT_MOTOR_PID_YAW_MIX_DIR      1.0F

#define SERVO_PID_ROLL_MIX               0.0F
#define SERVO_PID_PITCH_MIX              1.0F
#define SERVO_PID_YAW_MIX                0.0F

#define LEFT_SERVO_PID_ROLL_MIX_DIR      1.0F
#define LEFT_SERVO_PID_PITCH_MIX_DIR     1.0F
#define LEFT_SERVO_PID_YAW_MIX_DIR       1.0F

#define RIGHT_SERVO_PID_ROLL_MIX_DIR     1.0F
#define RIGHT_SERVO_PID_PITCH_MIX_DIR    1.0F
#define RIGHT_SERVO_PID_YAW_MIX_DIR      1.0F

#endif // CONF_H