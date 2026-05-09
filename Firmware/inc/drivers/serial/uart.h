#ifndef PERIPHS_UART_H
#define PERIPHS_UART_H

#include "hal.h"

#include "core/core.h"

#include "drivers/io/gpio.h"

#define UART_1_ID 0U
#define UART_2_ID 1U
#define UART_3_ID 2U
#define UART_4_ID 3U
#define UART_5_ID 4U

typedef uint8_t uart_id_t;

typedef struct {
    USART_TypeDef* pInstance;
    GPIO_TypeDef* pRx;
    GPIO_TypeDef* pTx;
    uint32_t rxPin;
    uint32_t txPin;
    uint32_t af;
    uint32_t irqId;
} UartHardware_t;

typedef void (*UartRxCallback_t) (uint8_t const* pData, uint32_t len);

typedef struct {
    uart_id_t id;
    UART_HandleTypeDef handle;
    UartHardware_t hardware;
    UartRxCallback_t rxCallback;
} Uart_t;

typedef struct UartPort_s {
    Uart_t* pUart;
    struct {
        uart_id_t id;
        uint32_t baudRate;
        UartRxCallback_t rxCallback;
        uint8_t irqPriority;
    } cfg;
} UartPort_t;

FJ_DECLARE_SHARED (Uart_t, g_Uarts[]);
FJ_DECLARE_SHARED (uint32_t, g_NumUarts);

eSTATUS_t Uart_InitSystem (void);
Uart_t* Uart_GetById (uart_id_t id);
eSTATUS_t UartPort_Init (UartPort_t* pOutPort);
eSTATUS_t UartPort_Write (UartPort_t* pPort, uint8_t const* pData, uint32_t size);

// clang-format on

#endif // PERIPHS_UART_H
