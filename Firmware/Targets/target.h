#ifndef TARGETS_TARGET_H
#define TARGETS_TARGET_H

#define TAR_NARG_(a1, a2, a3, a4, a5, a6, a7, a8, N, ...) N
#define TAR_NARG(...)                                     TAR_NARG_ (__VA_ARGS__, 8, 7, 6, 5, 4, 3, 2, 1, 0)
#define TAR_CONCAT(A, B)                                  A##B

#define TAR_GET_TIMER_INSTANCE(...)                       TAR_CONCAT (__VA_ARGS__, _INSTANCE)
#define TAR_GET_TIMER_AF(...)                             TAR_CONCAT (__VA_ARGS__, _AF)

#define MOTOR_1_ENABLED                                   0U
#define MOTOR_2_ENABLED                                   0U

#define TAR_MOTOR_COUNT()                                 (MOTOR_1_ENABLED + MOTOR_2_ENABLED)
#define TAR_GET_MOTOR_PORT(MOTOR_NAME)                    MOTOR_NAME##_GPIO_PORT
#define TAR_GET_MOTOR_PIN(MOTOR_NAME)                     MOTOR_NAME##_GPIO_PIN
#define TAR_GET_MOTOR_TIMER_NAME(MOTOR_NAME)              MOTOR_NAME##_TIMER
#define TAR_GET_MOTOR_TIMER_INSTANCE(MOTOR_NAME) \
    TAR_GET_TIMER_INSTANCE (TAR_GET_MOTOR_TIMER_NAME (MOTOR_NAME))
#define TAR_GET_MOTOR_TIMER_AF(MOTOR_NAME)      TAR_GET_TIMER_AF (TAR_GET_MOTOR_TIMER_NAME (MOTOR_NAME))
#define TAR_GET_MOTOR_TIMER_CHANNEL(MOTOR_NAME) MOTOR_NAME##_CHANNEL


#include "nucleo_h747zi.h"

#endif // TARGETS_TARGET_H