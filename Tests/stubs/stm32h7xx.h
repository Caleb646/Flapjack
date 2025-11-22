#ifndef STM32H7XX_H
#define STM32H7XX_H

#include <stdint.h>

#define USE_HAL_I2C_REGISTER_CALLBACKS  1
#define USE_HAL_SPI_REGISTER_CALLBACKS  1
#define USE_HAL_UART_REGISTER_CALLBACKS 1

#define __BKPT(arg)
#define __NOP()
#define __IO
#define CM7_CPUID    ((uint32_t)0x00000003)
#define CM4_CPUID    ((uint32_t)0x00000001)
#define CM4_SEV_IRQn ((uint32_t)0x00000001)
#define CM7_SEV_IRQn ((uint32_t)0x00000000)

extern uint32_t SystemCoreClock;

void HAL_Delay (uint32_t ms);
uint32_t HAL_GetTick (void);
uint32_t HAL_GetCurrentCPUID (void);
void __disable_irq (void);

typedef enum {
    DUMMY_IRQn = -14,
} IRQn_Type;

void HAL_NVIC_SetPriorityGrouping (uint32_t PriorityGroup);
void HAL_NVIC_SetPriority (IRQn_Type IRQn, uint32_t PreemptPriority, uint32_t SubPriority);
void HAL_NVIC_EnableIRQ (IRQn_Type IRQn);
void HAL_NVIC_DisableIRQ (IRQn_Type IRQn);
void HAL_NVIC_SystemReset (void);

typedef enum { RESET = 0, SET = !RESET } FlagStatus;
typedef enum { DISABLE = 0, ENABLE = !DISABLE } FunctionalState;

typedef struct {
    uint32_t MODER;   /*!< GPIO port mode register,               Address offset: 0x00      */
    uint32_t OTYPER;  /*!< GPIO port output type register,        Address offset: 0x04      */
    uint32_t OSPEEDR; /*!< GPIO port output speed register,       Address offset: 0x08      */
    uint32_t PUPDR;   /*!< GPIO port pull-up/pull-down register,  Address offset: 0x0C      */
    uint32_t IDR;     /*!< GPIO port input data register,         Address offset: 0x10      */
    uint32_t ODR;     /*!< GPIO port output data register,        Address offset: 0x14      */
    uint32_t BSRR;    /*!< GPIO port bit set/reset,               Address offset: 0x18      */
    uint32_t LCKR;    /*!< GPIO port configuration lock register, Address offset: 0x1C      */
    uint32_t AFR[2];  /*!< GPIO alternate function registers,     Address offset: 0x20-0x24 */
} GPIO_TypeDef;

extern GPIO_TypeDef g_GPIOA;
extern GPIO_TypeDef g_GPIOB;
extern GPIO_TypeDef g_GPIOC;
extern GPIO_TypeDef g_GPIOD;
extern GPIO_TypeDef g_GPIOE;
extern GPIO_TypeDef g_GPIOF;
extern GPIO_TypeDef g_GPIOG;
extern GPIO_TypeDef g_GPIOH;
extern GPIO_TypeDef g_GPIOI;
extern GPIO_TypeDef g_GPIOJ;
extern GPIO_TypeDef g_GPIOK;

#define GPIOA ((GPIO_TypeDef*)&g_GPIOA)
#define GPIOB ((GPIO_TypeDef*)&g_GPIOB)
#define GPIOC ((GPIO_TypeDef*)&g_GPIOC)
#define GPIOD ((GPIO_TypeDef*)&g_GPIOD)
#define GPIOE ((GPIO_TypeDef*)&g_GPIOE)
#define GPIOF ((GPIO_TypeDef*)&g_GPIOF)
#define GPIOG ((GPIO_TypeDef*)&g_GPIOG)
#define GPIOH ((GPIO_TypeDef*)&g_GPIOH)
#define GPIOI ((GPIO_TypeDef*)&g_GPIOI)
#define GPIOJ ((GPIO_TypeDef*)&g_GPIOJ)
#define GPIOK ((GPIO_TypeDef*)&g_GPIOK)

typedef struct {
    uint32_t CR1;       /*!< TIM control register 1,                   Address offset: 0x00 */
    uint32_t CR2;       /*!< TIM control register 2,                   Address offset: 0x04 */
    uint32_t SMCR;      /*!< TIM slave mode control register,          Address offset: 0x08 */
    uint32_t DIER;      /*!< TIM DMA/interrupt enable register,        Address offset: 0x0C */
    uint32_t SR;        /*!< TIM status register,                      Address offset: 0x10 */
    uint32_t EGR;       /*!< TIM event generation register,            Address offset: 0x14 */
    uint32_t CCMR1;     /*!< TIM capture/compare mode register 1,      Address offset: 0x18 */
    uint32_t CCMR2;     /*!< TIM capture/compare mode register 2,      Address offset: 0x1C */
    uint32_t CCER;      /*!< TIM capture/compare enable register,      Address offset: 0x20 */
    uint32_t CNT;       /*!< TIM counter register,                     Address offset: 0x24 */
    uint32_t PSC;       /*!< TIM prescaler,                            Address offset: 0x28 */
    uint32_t ARR;       /*!< TIM auto-reload register,                 Address offset: 0x2C */
    uint32_t RCR;       /*!< TIM repetition counter register,          Address offset: 0x30 */
    uint32_t CCR1;      /*!< TIM capture/compare register 1,           Address offset: 0x34 */
    uint32_t CCR2;      /*!< TIM capture/compare register 2,           Address offset: 0x38 */
    uint32_t CCR3;      /*!< TIM capture/compare register 3,           Address offset: 0x3C */
    uint32_t CCR4;      /*!< TIM capture/compare register 4,           Address offset: 0x40 */
    uint32_t BDTR;      /*!< TIM break and dead-time register,         Address offset: 0x44 */
    uint32_t DCR;       /*!< TIM DMA control register,                 Address offset: 0x48 */
    uint32_t DMAR;      /*!< TIM DMA address for full transfer,        Address offset: 0x4C */
    uint32_t RESERVED1; /*!< Reserved, 0x50                                                 */
    uint32_t CCMR3;     /*!< TIM capture/compare mode register 3,      Address offset: 0x54 */
    uint32_t CCR5;      /*!< TIM capture/compare register5,            Address offset: 0x58 */
    uint32_t CCR6;      /*!< TIM capture/compare register6,            Address offset: 0x5C */
    uint32_t AF1;       /*!< TIM alternate function option register 1, Address offset: 0x60 */
    uint32_t AF2;       /*!< TIM alternate function option register 2, Address offset: 0x64 */
    uint32_t TISEL;     /*!< TIM Input Selection register,             Address offset: 0x68 */
} TIM_TypeDef;

extern TIM_TypeDef g_TIM5;
extern TIM_TypeDef g_TIM8;
extern TIM_TypeDef g_TIM12;
extern TIM_TypeDef g_TIM13;

#define TIM5  ((TIM_TypeDef*)&g_TIM5)
#define TIM8  ((TIM_TypeDef*)&g_TIM8)
#define TIM12 ((TIM_TypeDef*)&g_TIM12)
#define TIM13 ((TIM_TypeDef*)&g_TIM13)

typedef struct {
    uint32_t CR1;  /*!< SPI/I2S Control register 1,                      Address offset: 0x00 */
    uint32_t CR2;  /*!< SPI Control register 2,                          Address offset: 0x04 */
    uint32_t CFG1; /*!< SPI Configuration register 1,                    Address offset: 0x08 */
    uint32_t CFG2; /*!< SPI Configuration register 2,                    Address offset: 0x0C */
    uint32_t IER;  /*!< SPI/I2S Interrupt Enable register,               Address offset: 0x10 */
    uint32_t SR;   /*!< SPI/I2S Status register,                         Address offset: 0x14 */
    uint32_t IFCR; /*!< SPI/I2S Interrupt/Status flags clear register,   Address offset: 0x18 */
    uint32_t RESERVED0; /*!< Reserved, 0x1C */
    uint32_t TXDR; /*!< SPI/I2S Transmit data register,                  Address offset: 0x20 */
    uint32_t RESERVED1[3]; /*!< Reserved, 0x24-0x2C */
    uint32_t RXDR; /*!< SPI/I2S Receive data register,                   Address offset: 0x30 */
    uint32_t RESERVED2[3]; /*!< Reserved, 0x34-0x3C */
    uint32_t CRCPOLY; /*!< SPI CRC Polynomial register,                     Address offset: 0x40 */
    uint32_t TXCRC;   /*!< SPI Transmitter CRC register,                    Address offset: 0x44 */
    uint32_t RXCRC;   /*!< SPI Receiver CRC register,                       Address offset: 0x48 */
    uint32_t UDRDR;   /*!< SPI Underrun data register,                      Address offset: 0x4C */
    uint32_t I2SCFGR; /*!< I2S Configuration register,                      Address offset: 0x50 */
} SPI_TypeDef;

extern SPI_TypeDef g_SPI1;
extern SPI_TypeDef g_SPI2;
extern SPI_TypeDef g_SPI3;
extern SPI_TypeDef g_SPI4;
extern SPI_TypeDef g_SPI5;

#define SPI1 ((SPI_TypeDef*)&g_SPI1)
#define SPI2 ((SPI_TypeDef*)&g_SPI2)
#define SPI3 ((SPI_TypeDef*)&g_SPI3)
#define SPI4 ((SPI_TypeDef*)&g_SPI4)
#define SPI5 ((SPI_TypeDef*)&g_SPI5)

typedef struct {
    uint32_t CR1;   /*!< USART Control register 1,                 Address offset: 0x00 */
    uint32_t CR2;   /*!< USART Control register 2,                 Address offset: 0x04 */
    uint32_t CR3;   /*!< USART Control register 3,                 Address offset: 0x08 */
    uint32_t BRR;   /*!< USART Baud rate register,                 Address offset: 0x0C */
    uint32_t GTPR;  /*!< USART Guard time and prescaler register,  Address offset: 0x10 */
    uint32_t RTOR;  /*!< USART Receiver Time Out register,         Address offset: 0x14 */
    uint32_t RQR;   /*!< USART Request register,                   Address offset: 0x18 */
    uint32_t ISR;   /*!< USART Interrupt and status register,      Address offset: 0x1C */
    uint32_t ICR;   /*!< USART Interrupt flag Clear register,      Address offset: 0x20 */
    uint32_t RDR;   /*!< USART Receive Data register,              Address offset: 0x24 */
    uint32_t TDR;   /*!< USART Transmit Data register,             Address offset: 0x28 */
    uint32_t PRESC; /*!< USART clock Prescaler register,           Address offset: 0x2C */
} USART_TypeDef;

extern USART_TypeDef g_USART1;
extern USART_TypeDef g_USART2;
extern USART_TypeDef g_USART3;
extern USART_TypeDef g_UART4;
extern USART_TypeDef g_UART5;
extern USART_TypeDef g_USART6;

#define USART1 ((USART_TypeDef*)&g_USART1)
#define USART2 ((USART_TypeDef*)&g_USART2)
#define USART3 ((USART_TypeDef*)&g_USART3)
#define UART4  ((USART_TypeDef*)&g_UART4)
#define UART5  ((USART_TypeDef*)&g_UART5)
#define USART6 ((USART_TypeDef*)&g_USART6)


typedef struct {
    uint32_t CR1;      /*!< I2C Control register 1,            Address offset: 0x00 */
    uint32_t CR2;      /*!< I2C Control register 2,            Address offset: 0x04 */
    uint32_t OAR1;     /*!< I2C Own address 1 register,        Address offset: 0x08 */
    uint32_t OAR2;     /*!< I2C Own address 2 register,        Address offset: 0x0C */
    uint32_t TIMINGR;  /*!< I2C Timing register,               Address offset: 0x10 */
    uint32_t TIMEOUTR; /*!< I2C Timeout register,              Address offset: 0x14 */
    uint32_t ISR;      /*!< I2C Interrupt and status register, Address offset: 0x18 */
    uint32_t ICR;      /*!< I2C Interrupt clear register,      Address offset: 0x1C */
    uint32_t PECR;     /*!< I2C PEC register,                  Address offset: 0x20 */
    uint32_t RXDR;     /*!< I2C Receive data register,         Address offset: 0x24 */
    uint32_t TXDR;     /*!< I2C Transmit data register,        Address offset: 0x28 */
} I2C_TypeDef;


typedef struct {
    __IO uint32_t CR;   /*!< DMA stream x configuration register      */
    __IO uint32_t NDTR; /*!< DMA stream x number of data register     */
    __IO uint32_t PAR;  /*!< DMA stream x peripheral address register */
    __IO uint32_t M0AR; /*!< DMA stream x memory 0 address register   */
    __IO uint32_t M1AR; /*!< DMA stream x memory 1 address register   */
    __IO uint32_t FCR;  /*!< DMA stream x FIFO control register       */
} DMA_Stream_TypeDef;

typedef struct {
    __IO uint32_t LISR;  /*!< DMA low interrupt status register,      Address offset: 0x00 */
    __IO uint32_t HISR;  /*!< DMA high interrupt status register,     Address offset: 0x04 */
    __IO uint32_t LIFCR; /*!< DMA low interrupt flag clear register,  Address offset: 0x08 */
    __IO uint32_t HIFCR; /*!< DMA high interrupt flag clear register, Address offset: 0x0C */
} DMA_TypeDef;

typedef struct {
    __IO uint32_t CCR;   /*!< DMA channel x configuration register          */
    __IO uint32_t CNDTR; /*!< DMA channel x number of data register         */
    __IO uint32_t CPAR;  /*!< DMA channel x peripheral address register     */
    __IO uint32_t CM0AR; /*!< DMA channel x memory 0 address register       */
    __IO uint32_t CM1AR; /*!< DMA channel x memory 1 address register       */
} BDMA_Channel_TypeDef;

typedef struct {
    __IO uint32_t ISR;  /*!< DMA interrupt status register,               Address offset: 0x00 */
    __IO uint32_t IFCR; /*!< DMA interrupt flag clear register,           Address offset: 0x04 */
} BDMA_TypeDef;

typedef struct {
    __IO uint32_t CCR; /*!< DMA Multiplexer Channel x Control Register   */
} DMAMUX_Channel_TypeDef;

typedef struct {
    __IO uint32_t CSR; /*!< DMA Channel Status Register     */
    __IO uint32_t CFR; /*!< DMA Channel Clear Flag Register */
} DMAMUX_ChannelStatus_TypeDef;

typedef struct {
    __IO uint32_t RGCR; /*!< DMA Request Generator x Control Register   */
} DMAMUX_RequestGen_TypeDef;

typedef struct {
    __IO uint32_t RGSR;  /*!< DMA Request Generator Status Register       */
    __IO uint32_t RGCFR; /*!< DMA Request Generator Clear Flag Register   */
} DMAMUX_RequestGenStatus_TypeDef;

/**
 * @brief MDMA Controller
 */
typedef struct {
    __IO uint32_t GISR0; /*!< MDMA Global Interrupt/Status Register 0,          Address offset: 0x00 */
} MDMA_TypeDef;

typedef struct {
    __IO uint32_t CISR; /*!< MDMA channel x interrupt/status register,             Address offset: 0x40 */
    __IO uint32_t CIFCR; /*!< MDMA channel x interrupt flag clear register,         Address offset: 0x44 */
    __IO uint32_t CESR; /*!< MDMA Channel x error status register,                 Address offset: 0x48 */
    __IO uint32_t CCR; /*!< MDMA channel x control register,                      Address offset: 0x4C */
    __IO uint32_t CTCR; /*!< MDMA channel x Transfer Configuration register,       Address offset: 0x50 */
    __IO uint32_t CBNDTR; /*!< MDMA Channel x block number of data register,         Address offset: 0x54 */
    __IO uint32_t CSAR; /*!< MDMA channel x source address register,               Address offset: 0x58 */
    __IO uint32_t CDAR; /*!< MDMA channel x destination address register,          Address offset: 0x5C */
    __IO uint32_t CBRUR; /*!< MDMA channel x Block Repeat address Update register,  Address offset: 0x60 */
    __IO uint32_t CLAR; /*!< MDMA channel x Link Address register,                 Address offset: 0x64 */
    __IO uint32_t CTBR; /*!< MDMA channel x Trigger and Bus selection Register,    Address offset: 0x68 */
    uint32_t RESERVED0; /*!< Reserved, 0x6C */
    __IO uint32_t CMAR; /*!< MDMA channel x Mask address register,                 Address offset: 0x70 */
    __IO uint32_t CMDR; /*!< MDMA channel x Mask Data register,                    Address offset: 0x74 */
} MDMA_Channel_TypeDef;

extern DMA_Stream_TypeDef g_DMA1_Stream0;
extern DMA_Stream_TypeDef g_DMA1_Stream1;
extern DMA_Stream_TypeDef g_DMA1_Stream2;
extern DMA_Stream_TypeDef g_DMA1_Stream3;
extern DMA_Stream_TypeDef g_DMA1_Stream4;
extern DMA_Stream_TypeDef g_DMA1_Stream5;
extern DMA_Stream_TypeDef g_DMA1_Stream6;
extern DMA_Stream_TypeDef g_DMA1_Stream7;

#define DMA1_Stream0 (&g_DMA1_Stream0)
#define DMA1_Stream1 (&g_DMA1_Stream1)
#define DMA1_Stream2 (&g_DMA1_Stream2)
#define DMA1_Stream3 (&g_DMA1_Stream3)
#define DMA1_Stream4 (&g_DMA1_Stream4)
#define DMA1_Stream5 (&g_DMA1_Stream5)
#define DMA1_Stream6 (&g_DMA1_Stream6)
#define DMA1_Stream7 (&g_DMA1_Stream7)


#endif // STM32H7XX_H