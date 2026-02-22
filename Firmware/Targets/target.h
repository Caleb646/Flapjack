#ifndef TARGETS_TARGET_H
#define TARGETS_TARGET_H

// to disable redefinition warnings for default target macros
#pragma GCC system_header

// #define BRD_STRINGIFY_(x)                                 "" #x
// #define BRD_STRINGIFY(...)                                BRD_STRINGIFY_ (__VA_ARGS__)
#define BRD_NARG_(a1, a2, a3, a4, a5, a6, a7, a8, N, ...) N
#define BRD_NARG(...)                                     BRD_NARG_ (__VA_ARGS__, 8, 7, 6, 5, 4, 3, 2, 1, 0)
#define BRD_CONCAT_(A, B)                                 A##B
#define BRD_CONCAT(...)                                   BRD_CONCAT_ (__VA_ARGS__)

// #define BRD_IS_ENABLED_(NAME)                             ({ (__builtin_strcmp ("" #NAME, BRD_STRINGIFY_ (NAME)) == 0); })
// #define BRD_IS_ENABLED__(...)                             BRD_IS_ENABLED_ (__VA_ARGS__)
// #define BRD_IS_ENABLED(NAME)                              BRD_IS_ENABLED__ (BRD_CONCAT (NAME, _ENABLED))

#define BRD_GET_NAME(API, PERIPH)                         BRD_CONCAT (API, _##PERIPH)
#define BRD_GET_2(API, ATTR)                              BRD_CONCAT (API, _##ATTR)
#define BRD_GET_3(API, PERIPH, ATTR)                      BRD_CONCAT (BRD_GET_NAME (API, PERIPH), _##ATTR)
#define BRD_GET_ATTR(...)                                 BRD_CONCAT (BRD_GET_, BRD_NARG (__VA_ARGS__)) (__VA_ARGS__)

#define BRD_GET_ID(...)                                   BRD_GET_ATTR (__VA_ARGS__, _ID)
#define BRD_GET_INSTANCE(...)                             BRD_GET_ATTR (__VA_ARGS__, INSTANCE)
#define BRD_GET_GPIO_PORT(...)                            BRD_GET_ATTR (__VA_ARGS__, GPIO_PORT)
#define BRD_GET_GPIO_PIN(...)                             BRD_GET_ATTR (__VA_ARGS__, GPIO_PIN)
#define BRD_GET_AF(...)                                   BRD_GET_ATTR (__VA_ARGS__, AF)
#define BRD_GET_BAUD_RATE(...)                            BRD_GET_ATTR (__VA_ARGS__, BAUD_RATE)

#define BRD_GET_UART_ID(...)                              BRD_CONCAT (BRD_GET_ATTR (__VA_ARGS__, UART), _ID)

#define BRD_GET_TIMER_INSTANCE(...)                       BRD_GET_ATTR (__VA_ARGS__, TIMER, INSTANCE)
#define BRD_GET_TIMER_CHANNEL(...)                        BRD_GET_ATTR (__VA_ARGS__, CHANNEL)
#define BRD_GET_TIMER_AF(...)                             BRD_GET_ATTR (__VA_ARGS__, TIMER, AF)

#define MOTOR_1_ENABLED                                   0U
#define MOTOR_2_ENABLED                                   0U
#define BRD_MOTOR_COUNT                                   (MOTOR_1_ENABLED + MOTOR_2_ENABLED)


#define SERVO_1_ENABLED                                   0U
#define SERVO_2_ENABLED                                   0U
#define BRD_SERVO_COUNT                                   (SERVO_1_ENABLED + SERVO_2_ENABLED)

#if defined(BOARD_NUCLEO_H747ZI)
#include "boards/nucleo_h747zi.h"
#elif defined(BOARD_FLAPJACK_V1)
#include "boards/flapjack_v1.h"
#else
#error "Invalid board"
#endif
#include "cfgs/cfg.h"

#endif // TARGETS_TARGET_H