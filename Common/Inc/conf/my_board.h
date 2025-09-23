#ifndef CONF_MY_BOARD_H
#define CONF_MY_BOARD_H

#include "conf/ids.h"
#include "hal.h"

#define IMU_INT_EXTI_IRQn                EXTI9_5_IRQn
#define IMU_INT_GPIO_Pin                 GPIO_PIN_5
#define IMU_BUS_ID                       eSPI_1_BUS_ID

#define MAG_INT_EXTI_IRQn                EXTI3_IRQn
#define MAG_INT_GPIO_Pin                 GPIO_PIN_3
#define MAG_BUS_ID                       eSPI_5_BUS_ID

#define BARO_INT_EXTI_IRQn               EXTI9_5_IRQn
#define BARO_INT_GPIO_Pin                GPIO_PIN_6
#define BARO_BUS_ID                      eSPI_5_BUS_ID

#define FLASH_BUS_ID                     eSPI_3_BUS_ID

#define DEBUG_BUS_ID                     eUART_1_BUS_ID

#define GPS_BUS_ID                       eUART_2_BUS_ID

#define RF_RECEIVER_BUS_ID               eUART_3_BUS_ID

#define LEFT_SERVO_MOTOR_PWM_TIMER       eTIMER_5_ID
#define LEFT_SERVO_MOTOR_PWM_CHANNEL     TIM_CHANNEL_1
#define LEFT_SERVO_MOTOR_PWM_FREQUENCY   50U // 50 Hz

#define LEFT_SERVO_AILERON_PWM_TIMER     eTIMER_5_ID
#define LEFT_SERVO_AILERON_PWM_CHANNEL   TIM_CHANNEL_2
#define LEFT_SERVO_AILERON_PWM_FREQUENCY 50U // 50 Hz

#define LEFT_SERVO_3_PWM_TIMER           eTIMER_5_ID
#define LEFT_SERVO_3_PWM_CHANNEL         TIM_CHANNEL_3
#define LEFT_SERVO_3_PWM_FREQUENCY       50U // 50 Hz

#define LEFT_SERVO_4_PWM_TIMER           eTIMER_5_ID
#define LEFT_SERVO_4_PWM_CHANNEL         TIM_CHANNEL_4
#define LEFT_SERVO_4_PWM_FREQUENCY       50U // 50 Hz

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

#define LEFT_MOTOR_PWM_TIMER             eTIMER_12_ID
#define LEFT_MOTOR_PWM_CHANNEL           TIM_CHANNEL_1

#define RIGHT_MOTOR_PWM_TIMER            eTIMER_12_ID
#define RIGHT_MOTOR_PWM_CHANNEL          TIM_CHANNEL_2

#endif // CONF_MY_BOARD_H