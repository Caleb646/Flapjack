#ifndef TARGETS_TARGET_H
#define TARGETS_TARGET_H

#define TAR_NARG_(a1, a2, a3, a4, a5, a6, a7, a8, N, ...) N
#define TAR_NARG(...)                                     TAR_NARG_ (__VA_ARGS__, 8, 7, 6, 5, 4, 3, 2, 1, 0)
#define TAR_CONCAT_(A, B)                                 A##B
#define BRD_CONCAT(...)                                   TAR_CONCAT_ (__VA_ARGS__)

#define TAR_GET_TIMER_INSTANCE(...)                       TAR_CONCAT_ (__VA_ARGS__, _INSTANCE)
#define TAR_GET_TIMER_AF(...)                             TAR_CONCAT_ (__VA_ARGS__, _AF)

#define MOTOR_1_ENABLED                                   0U
#define MOTOR_2_ENABLED                                   0U

#define BRD_GET_NAME(API, PERIPH)                         BRD_CONCAT (API, _##PERIPH)
#define BRD_GET_ID(API, PERIPH)                           BRD_CONCAT (BRD_GET_NAME (API, PERIPH), _ID)
#define BRD_GET_ATTRIBUTE(API, PERIPH, ATTR)              BRD_CONCAT (BRD_GET_NAME (API, PERIPH), _##ATTR)
#define BRD_GET_INSTANCE(API, PERIPH)                     BRD_GET_ATTRIBUTE (API, PERIPH, INSTANCE)
#define BRD_GET_GPIO_PORT(API, PERIPH)                    BRD_GET_ATTRIBUTE (API, PERIPH, GPIO_PORT)
#define BRD_GET_GPIO_PIN(API, PERIPH)                     BRD_GET_ATTRIBUTE (API, PERIPH, GPIO_PIN)
#define BRD_GET_AF(API, PERIPH)                           BRD_GET_ATTRIBUTE (API, PERIPH, AF)
#define BRD_GET_BAUD_RATE(API, PERIPH)                    BRD_GET_ATTRIBUTE (API, PERIPH, BAUD_RATE)

#define BRD_MOTOR_COUNT                                   (MOTOR_1_ENABLED + MOTOR_2_ENABLED)
#define TAR_GET_MOTOR_PORT(MOTOR_NAME)                    MOTOR_NAME##_GPIO_PORT
#define TAR_GET_MOTOR_PIN(MOTOR_NAME)                     MOTOR_NAME##_GPIO_PIN
#define TAR_GET_MOTOR_TIMER_NAME(MOTOR_NAME)              MOTOR_NAME##_TIMER
#define TAR_GET_MOTOR_TIMER_INSTANCE(MOTOR_NAME) \
    TAR_GET_TIMER_INSTANCE (TAR_GET_MOTOR_TIMER_NAME (MOTOR_NAME))
#define TAR_GET_MOTOR_TIMER_AF(MOTOR_NAME)      TAR_GET_TIMER_AF (TAR_GET_MOTOR_TIMER_NAME (MOTOR_NAME))
#define TAR_GET_MOTOR_TIMER_CHANNEL(MOTOR_NAME) MOTOR_NAME##_CHANNEL

#define SERVO_1_ENABLED                         0U
#define SERVO_2_ENABLED                         0U

#define BRD_SERVO_COUNT                         (SERVO_1_ENABLED + SERVO_2_ENABLED)
#define BRD_GET_SERVO_PORT(SERVO_NAME)          SERVO_NAME##_GPIO_PORT
#define BRD_GET_SERVO_PIN(SERVO_NAME)           SERVO_NAME##_GPIO_PIN
#define BRD_GET_SERVO_TIMER_NAME(SERVO_NAME)    SERVO_NAME##_TIMER
#define BRD_GET_SERVO_TIMER_INSTANCE(SERVO_NAME) \
    TAR_GET_TIMER_INSTANCE (BRD_GET_SERVO_TIMER_NAME (SERVO_NAME))
#define BRD_GET_SERVO_TIMER_AF(SERVO_NAME)      TAR_GET_TIMER_AF (BRD_GET_SERVO_TIMER_NAME (SERVO_NAME))
#define BRD_GET_SERVO_TIMER_CHANNEL(SERVO_NAME) SERVO_NAME##_CHANNEL


#include "nucleo_h747zi.h"

#endif // TARGETS_TARGET_H