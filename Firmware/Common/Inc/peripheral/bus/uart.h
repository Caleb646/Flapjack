#ifndef PERIPHS_UART_H
#define PERIPHS_UART_H
#include "conf/board.h"
#include "conf/conf.h"
#include "core/core.h"
#include "hal.h"
#include "peripheral/bus/bus_core.h"
#include "peripheral/gpio.h"

#define UART_1_ID 0U
#define UART_2_ID 1U
#define UART_3_ID 2U
#define UART_4_ID 3U
#define UART_5_ID 4U
#define UART_6_ID 5U

typedef uint8_t uart_id_t;

typedef struct {
    USART_TypeDef* pInstance;
    GPIO_TypeDef* pRx;
    GPIO_TypeDef* pTx;
    uint16_t rxPin;
    uint16_t txPin;
    uint32_t af;
} UartHardware_t;

typedef struct {
    uart_id_t id;
    UART_HandleTypeDef handle;
    UartHardware_t hardware;
} Uart_t;

typedef struct {
    Uart_t* pUart;
    struct {
        uart_id_t id;
        uint32_t baudRate;
    } cfg;
} UartPort_t;

FJ_DECLARE_SHARED (Uart_t, g_Uarts[]);
FJ_DECLARE_SHARED (uint32_t, g_numUarts);

eSTATUS_t Uart_InitSystem (void);
eSTATUS_t UartPort_Init (UartPort_t* pOutPort);
eSTATUS_t UartPort_Write (UartPort_t* pPort, uint8_t const* pData, uint32_t size);

// clang-format on

#endif // PERIPHS_UART_H
