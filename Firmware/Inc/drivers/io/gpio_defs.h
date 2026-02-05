#ifndef DRIVERS_GPIO_DEFS_H
#define DRIVERS_GPIO_DEFS_H

#include <stdint.h>

typedef uint8_t eGPIO_PINID_t;
#define eGPIO_PINID_NONE            0U
#define eGPIO_PINID_0               1U
#define eGPIO_PINID_1               2U
#define eGPIO_PINID_2               3U
#define eGPIO_PINID_3               4U
#define eGPIO_PINID_4               5U
#define eGPIO_PINID_5               6U
#define eGPIO_PINID_6               7U
#define eGPIO_PINID_7               8U
#define eGPIO_PINID_8               9U
#define eGPIO_PINID_9               10U
#define eGPIO_PINID_10              11U
#define eGPIO_PINID_11              12U
#define eGPIO_PINID_12              13U
#define eGPIO_PINID_13              14U
#define eGPIO_PINID_14              15U
#define eGPIO_PINID_15              16U
#define GPIO_PINID_TO_INDEX(PIN_ID) ((PIN_ID) - 1U)

typedef uint8_t eGPIO_PORTID_t;
#define eGPIO_PORTID_NONE             0U
#define eGPIO_PORTID_A                1U
#define eGPIO_PORTID_B                2U
#define eGPIO_PORTID_C                3U
#define eGPIO_PORTID_D                4U
#define eGPIO_PORTID_E                5U
#define eGPIO_PORTID_F                6U
#define eGPIO_PORTID_G                7U
#define eGPIO_PORTID_H                8U
#define eGPIO_PORTID_I                9U
#define eGPIO_PORTID_J                10U
#define eGPIO_PORTID_K                11U
#define GPIO_PORTID_TO_INDEX(PORT_ID) ((PORT_ID) - 1U)

typedef uint8_t eGPIO_ID_t;
#define I_GPIO_ID_MAKE(PORT_ID, PIN_ID)                 ((eGPIO_ID_t)(PORT_ID) << 4U | (eGPIO_ID_t)(PIN_ID))
#define I_GPIO_ID_MAKE_EXPAND(PORT, PIN)                I_GPIO_ID_MAKE (eGPIO_PORTID_##PORT, eGPIO_PINID_##PIN)
#define GPIO_ID_MAKE(...)                               I_GPIO_ID_MAKE_EXPAND (__VA_ARGS__)
#define GPIO_ID_TO_PORT_INDEX(GPIO_ID)                  (((GPIO_ID) >> 4U) - 1U)
#define GPIO_ID_TO_PIN_INDEX(GPIO_ID)                   (((GPIO_ID) & 0x0FU) - 1U)

#define GPIO_GET_ARG_COUNT(_0, _1, _2, N, ...)      N
#define GPIO_COUNT_ARGS(...)                        GPIO_GET_ARG_COUNT (, ##__VA_ARGS__, 2, 1, 0)
#define GPIO_CONCAT(a, b)                           a##b
#define GPIO_CONCAT_EXPAND(a, b)                    GPIO_CONCAT (a, b)
#define GPIO_ID_MAKE_OR_INVALID_0()                 eGPIO_PORTID_NONE
#define GPIO_ID_MAKE_OR_INVALID_1(arg1)             eGPIO_PORTID_NONE
#define GPIO_ID_MAKE_OR_INVALID_2(port, pin)        GPIO_ID_MAKE (port, pin)
#define GPIO_ID_MAKE_OR_INVALID(...) \
    GPIO_CONCAT_EXPAND (GPIO_ID_MAKE_OR_INVALID_, GPIO_COUNT_ARGS (__VA_ARGS__)) (__VA_ARGS__)

#define I_GPIO_ID_MAKE_PORTID_EXPAND(PORTNAME, ...) (eGPIO_PORTID_##PORTNAME)
#define GPIO_ID_MAKE_PORTID(...)                    I_GPIO_ID_MAKE_PORTID_EXPAND (__VA_ARGS__)

#define I_GPIO_ID_MAKE_PINID_EXPAND(PINNUM)         (eGPIO_PINID_##PINNUM)
#define GPIO_ID_MAKE_PINID(...)                     I_GPIO_ID_MAKE_PINID_EXPAND (__VA_ARGS__)

typedef struct {
    uint32_t mode;
    uint8_t pull;
    uint8_t speed;
} GPIOCfg_t;

typedef struct {
    // void* pPort;
    // uint16_t pin;
    eGPIO_ID_t id;
    uint8_t ownerId;
} GPIO_t;

typedef uint8_t eGPIO_STATE_t;
enum { eGPIO_STATE_LOW = 0U, eGPIO_STATE_HIGH = 1U };


#endif // DRIVERS_GPIO_DEFS_H