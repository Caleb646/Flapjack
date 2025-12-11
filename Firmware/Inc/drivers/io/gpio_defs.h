#ifndef DRIVERS_GPIO_DEFS_H
#define DRIVERS_GPIO_DEFS_H

#include <stdint.h>

typedef uint8_t eGPIO_PINID_t;
enum {
    eGPIO_PINID_NULL = 0U,
    eGPIO_PINID_0    = 1U,
    eGPIO_PINID_1    = 2U,
    eGPIO_PINID_2    = 3U,
    eGPIO_PINID_3    = 4U,
    eGPIO_PINID_4    = 5U,
    eGPIO_PINID_5    = 6U,
    eGPIO_PINID_6    = 7U,
    eGPIO_PINID_7    = 8U,
    eGPIO_PINID_8    = 9U,
    eGPIO_PINID_9    = 10U,
    eGPIO_PINID_10   = 11U,
    eGPIO_PINID_11   = 12U,
    eGPIO_PINID_12   = 13U,
    eGPIO_PINID_13   = 14U,
    eGPIO_PINID_14   = 15U,
    eGPIO_PINID_15   = 16U
};

#define GPIO_PIN_ID_TO_INDEX(PIN_ID) ((PIN_ID) - 1U)

typedef uint8_t eGPIO_PORTID_t;
enum {
    eGPIO_PORTID_NULL = 0U,
    eGPIO_PORTID_A    = 1U,
    eGPIO_PORTID_B    = 2U,
    eGPIO_PORTID_C    = 3U,
    eGPIO_PORTID_D    = 4U,
    eGPIO_PORTID_E    = 5U,
    eGPIO_PORTID_F    = 6U,
    eGPIO_PORTID_G    = 7U,
    eGPIO_PORTID_H    = 8U,
    eGPIO_PORTID_I    = 9U,
    eGPIO_PORTID_J    = 10U,
    eGPIO_PORTID_K    = 11U
};

#define GPIO_PORT_ID_TO_INDEX(PORT_ID) ((PORT_ID) - 1U)

typedef uint8_t eGPIO_ID_t;

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