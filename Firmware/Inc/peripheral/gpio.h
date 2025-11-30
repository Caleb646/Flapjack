#ifndef PERIPHS_GPIO_H
#define PERIPHS_GPIO_H

#include "common.h"
#include "conf/board.h"
#include "conf/conf.h"
#include "hal.h"

#define GPIO_VALID(pIO)     ((pIO) != NULL && (pIO)->pPort != NULL)
#define GPIO_HAS_OWNER(pIO) ((pIO)->ownerId != eDEVICE_ID_NULL)
#define GPIO_WRITE_PIN(pPORT, PIN, STATE)                                         \
    do {                                                                          \
        (pPORT)->BSRR = ((uint32_t)(PIN) << (16U * ((STATE) == GPIO_PIN_RESET))); \
    } while (0)

typedef struct {
    GPIO_TypeDef* pPort;
    uint16_t pin;
    eDEVICE_ID_t ownerId;
} IO_t;

// typedef IO_t volatile vIO_t;
typedef IO_t vIO_t;

#ifdef UNIT_TEST

void GPIOGetIOs (IO_t** ppIOs, uint32_t* pCount);

#endif // UNIT_TEST

eSTATUS_t GPIOSystemInit (void);
vIO_t* GPIOGetIOfromId (eGPIO_ID_t gpioId);
eSTATUS_t GPIOFreeById (eGPIO_ID_t gpioId);
eSTATUS_t GPIOFreeByIO (vIO_t* pIO);
eSTATUS_t GPIOInit (eDEVICE_ID_t ownerId, GPIODesc_t* pGPIODesc);

#define GPIO_INIT(OWNER_ID, pGPIO_DESC) GPIOInit ((OWNER_ID), (pGPIO_DESC));

#endif // PERIPHS_GPIO_H