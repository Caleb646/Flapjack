#ifndef HAL_H
#define HAL_H

#ifndef UNIT_TEST

// #include "cmsis_os.h"
// #include "cmsis_gcc.h"
#include "stm32h7xx.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_ll_spi.h"

#endif


#ifdef UNIT_TEST

// #include "../Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_hal_dma.h"
// #include "stm32h7xx_hal_dma.h"
#include <stdint.h>

#define __BKPT(arg)
#define __NOP()
#define CM7_CPUID    ((uint32_t)0x00000003)
#define CM4_CPUID    ((uint32_t)0x00000001)
#define CM4_SEV_IRQn ((uint32_t)0x00000001)
#define CM7_SEV_IRQn ((uint32_t)0x00000000)

typedef enum {
    HAL_OK      = 0x00,
    HAL_ERROR   = 0x01,
    HAL_BUSY    = 0x02,
    HAL_TIMEOUT = 0x03
} HAL_StatusTypeDef;

void HAL_Delay (uint32_t ms);
uint32_t HAL_GetTick (void);
uint32_t HAL_GetCurrentCPUID (void);
void __disable_irq (void);

extern uint32_t SystemCoreClock;

#define DMA_PERIPH_TO_MEMORY             ((uint32_t)UINT32_MAX)
#define DMA_MEMORY_TO_PERIPH             ((uint32_t)UINT32_MAX - 1U)
#define DMA_MEMORY_TO_MEMORY             ((uint32_t)UINT32_MAX - 2U)

#define DMA_PINC_ENABLE                  ((uint32_t)UINT32_MAX)
#define DMA_PINC_DISABLE                 ((uint32_t)UINT32_MAX - 1U)

#define DMA_MINC_ENABLE                  ((uint32_t)UINT32_MAX)
#define DMA_MINC_DISABLE                 ((uint32_t)UINT32_MAX - 1U)

#define DMA_PDATAALIGN_BYTE              ((uint32_t)UINT32_MAX)
#define DMA_PDATAALIGN_HALFWORD          ((uint32_t)UINT32_MAX - 1U)
#define DMA_PDATAALIGN_WORD              ((uint32_t)UINT32_MAX - 2U)

#define DMA_MDATAALIGN_BYTE              ((uint32_t)UINT32_MAX)
#define DMA_MDATAALIGN_HALFWORD          ((uint32_t)UINT32_MAX - 1U)
#define DMA_MDATAALIGN_WORD              ((uint32_t)UINT32_MAX - 2U)

#define DMA_NORMAL                       ((uint32_t)UINT32_MAX)
#define DMA_CIRCULAR                     ((uint32_t)UINT32_MAX - 1U)
#define DMA_PFCTRL                       ((uint32_t)UINT32_MAX - 2U)
#define DMA_DOUBLE_BUFFER_M0             ((uint32_t)UINT32_MAX - 3U)
#define DMA_DOUBLE_BUFFER_M1             ((uint32_t)(UINT32_MAX - 4U))

#define DMA_PRIORITY_LOW                 ((uint32_t)UINT32_MAX)
#define DMA_PRIORITY_MEDIUM              ((uint32_t)UINT32_MAX - 1U)
#define DMA_PRIORITY_HIGH                ((uint32_t)UINT32_MAX - 2U)
#define DMA_PRIORITY_VERY_HIGH           ((uint32_t)UINT32_MAX - 3U)

#define DMA_FIFOMODE_DISABLE             ((uint32_t)UINT32_MAX - 4U)
#define DMA_FIFOMODE_ENABLE              ((uint32_t)UINT32_MAX - 5U)

#define DMA_FIFO_THRESHOLD_1QUARTERFULL  ((uint32_t)UINT32_MAX - 6U)
#define DMA_FIFO_THRESHOLD_HALFFULL      ((uint32_t)UINT32_MAX - 7U)
#define DMA_FIFO_THRESHOLD_3QUARTERSFULL ((uint32_t)UINT32_MAX - 8U)
#define DMA_FIFO_THRESHOLD_FULL          ((uint32_t)UINT32_MAX - 9U)

#define DMA_MBURST_SINGLE                ((uint32_t)UINT32_MAX - 10U)
#define DMA_MBURST_INC4                  ((uint32_t)UINT32_MAX - 11U)
#define DMA_MBURST_INC8                  ((uint32_t)UINT32_MAX - 12U)
#define DMA_MBURST_INC16                 ((uint32_t)UINT32_MAX - 13U)

#define DMA_PBURST_SINGLE                ((uint32_t)UINT32_MAX - 14U)
#define DMA_PBURST_INC4                  ((uint32_t)UINT32_MAX - 15U)
#define DMA_PBURST_INC8                  ((uint32_t)UINT32_MAX - 16U)
#define DMA_PBURST_INC16                 ((uint32_t)UINT32_MAX - 17U)

#define DMA1_Stream0_IRQn                (1U)
#define DMA1_Stream1_IRQn                (2U)
#define DMA1_Stream2_IRQn                (3U)
#define DMA1_Stream3_IRQn                (4U)
#define DMA1_Stream4_IRQn                (5U)
#define DMA1_Stream5_IRQn                (6U)
#define DMA1_Stream6_IRQn                (7U)

#define __HAL_RCC_DMA1_CLK_ENABLE()

typedef struct {
    uint32_t Request;
    uint32_t Direction;
    uint32_t PeriphInc;
    uint32_t MemInc;
    uint32_t PeriphDataAlignment;
    uint32_t MemDataAlignment;
    uint32_t Mode;
    uint32_t Priority;
    uint32_t FIFOMode;
    uint32_t FIFOThreshold;
    uint32_t MemBurst;
    uint32_t PeriphBurst;
} DMA_InitTypeDef;

typedef struct {
    void* Instance;
    DMA_InitTypeDef Init;
    void* Parent;
} DMA_HandleTypeDef;

typedef struct {
    uint32_t CR;
    uint32_t NDTR;
    uint32_t PAR;
    uint32_t M0AR;
    uint32_t M1AR;
    uint32_t FCR;
} DMA_Stream_TypeDef;

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

// clang-format off

void HAL_DMA_IRQHandler(DMA_HandleTypeDef *hdma);
HAL_StatusTypeDef HAL_DMA_Init(DMA_HandleTypeDef *hdma);
HAL_StatusTypeDef HAL_DMA_DeInit(DMA_HandleTypeDef *hdma);

typedef struct
{
  uint32_t MODER;  
  uint32_t OTYPER; 
  uint32_t OSPEEDR;
  uint32_t PUPDR;  
  uint32_t IDR;    
  uint32_t ODR;    
  uint32_t BSRR;   
  uint32_t LCKR;   
  uint32_t AFR[2]; 
} GPIO_TypeDef;

typedef struct
{
  uint32_t Pin;                           
  uint32_t Mode;       
  uint32_t Speed;      
  uint32_t Pull;       
  uint32_t Alternate;                 
} GPIO_InitTypeDef;

typedef enum
{
  GPIO_PIN_RESET = 0U,
  GPIO_PIN_SET
} GPIO_PinState;

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

#define GPIOA               ((GPIO_TypeDef *) &g_GPIOA)
#define GPIOB               ((GPIO_TypeDef *) &g_GPIOB)
#define GPIOC               ((GPIO_TypeDef *) &g_GPIOC)
#define GPIOD               ((GPIO_TypeDef *) &g_GPIOD)
#define GPIOE               ((GPIO_TypeDef *) &g_GPIOE)
#define GPIOF               ((GPIO_TypeDef *) &g_GPIOF)
#define GPIOG               ((GPIO_TypeDef *) &g_GPIOG)
#define GPIOH               ((GPIO_TypeDef *) &g_GPIOH)
#define GPIOI               ((GPIO_TypeDef *) &g_GPIOI)
#define GPIOJ               ((GPIO_TypeDef *) &g_GPIOJ)
#define GPIOK               ((GPIO_TypeDef *) &g_GPIOK)

#define GPIO_PIN_0                 ((uint16_t)0x0001)
#define GPIO_PIN_1                 ((uint16_t)0x0002)
#define GPIO_PIN_2                 ((uint16_t)0x0004)
#define GPIO_PIN_3                 ((uint16_t)0x0008)
#define GPIO_PIN_4                 ((uint16_t)0x0010)
#define GPIO_PIN_5                 ((uint16_t)0x0020)
#define GPIO_PIN_6                 ((uint16_t)0x0040)
#define GPIO_PIN_7                 ((uint16_t)0x0080)
#define GPIO_PIN_8                 ((uint16_t)0x0100)
#define GPIO_PIN_9                 ((uint16_t)0x0200)
#define GPIO_PIN_10                ((uint16_t)0x0400)
#define GPIO_PIN_11                ((uint16_t)0x0800)
#define GPIO_PIN_12                ((uint16_t)0x1000)
#define GPIO_PIN_13                ((uint16_t)0x2000)
#define GPIO_PIN_14                ((uint16_t)0x4000)
#define GPIO_PIN_15                ((uint16_t)0x8000)
#define GPIO_PIN_All               ((uint16_t)0xFFFF)

#define __HAL_RCC_GPIOA_CLK_ENABLE()
#define __HAL_RCC_GPIOB_CLK_ENABLE()
#define __HAL_RCC_GPIOC_CLK_ENABLE()
#define __HAL_RCC_GPIOD_CLK_ENABLE()
#define __HAL_RCC_GPIOE_CLK_ENABLE()
#define __HAL_RCC_GPIOF_CLK_ENABLE()
#define __HAL_RCC_GPIOG_CLK_ENABLE()
#define __HAL_RCC_GPIOH_CLK_ENABLE()
#define __HAL_RCC_GPIOI_CLK_ENABLE()
#define __HAL_RCC_GPIOJ_CLK_ENABLE()
#define __HAL_RCC_GPIOK_CLK_ENABLE()

void HAL_GPIO_Init(GPIO_TypeDef  *GPIOx, const GPIO_InitTypeDef *GPIO_Init);
void HAL_GPIO_DeInit(GPIO_TypeDef  *GPIOx, uint32_t GPIO_Pin);
void HAL_GPIO_WritePin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, uint32_t PinState);
void HAL_GPIO_TogglePin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);

typedef enum
{
  HAL_TIM_ACTIVE_CHANNEL_1        = 0x01U,
  HAL_TIM_ACTIVE_CHANNEL_2        = 0x02U,
  HAL_TIM_ACTIVE_CHANNEL_3        = 0x04U,
  HAL_TIM_ACTIVE_CHANNEL_4        = 0x08U,
  HAL_TIM_ACTIVE_CHANNEL_5        = 0x10U,
  HAL_TIM_ACTIVE_CHANNEL_6        = 0x20U,
  HAL_TIM_ACTIVE_CHANNEL_CLEARED  = 0x00U 
} HAL_TIM_ActiveChannel;

typedef struct
{
  uint32_t CR1;      
  uint32_t CR2;      
  uint32_t SMCR;     
  uint32_t DIER;     
  uint32_t SR;       
  uint32_t EGR;      
  uint32_t CCMR1;    
  uint32_t CCMR2;    
  uint32_t CCER;     
  uint32_t CNT;      
  uint32_t PSC;      
  uint32_t ARR;      
  uint32_t RCR;      
  uint32_t CCR1;     
  uint32_t CCR2;     
  uint32_t CCR3;     
  uint32_t CCR4;     
  uint32_t BDTR;     
  uint32_t DCR;      
  uint32_t DMAR;     
  uint32_t      RESERVED1;
  uint32_t CCMR3;    
  uint32_t CCR5;     
  uint32_t CCR6;     
  uint32_t AF1;      
  uint32_t AF2;      
  uint32_t TISEL;    
} TIM_TypeDef;

extern TIM_TypeDef g_TIM5;
extern TIM_TypeDef g_TIM8;
extern TIM_TypeDef g_TIM12;
extern TIM_TypeDef g_TIM13;

#define TIM5                ((TIM_TypeDef *) &g_TIM5)
#define TIM8                ((TIM_TypeDef *) &g_TIM8)
#define TIM12               ((TIM_TypeDef *) &g_TIM12)
#define TIM13               ((TIM_TypeDef *) &g_TIM13)

#define DMA_REQUEST_TIM8_CH1         47U  
#define DMA_REQUEST_TIM8_CH2         48U  
#define DMA_REQUEST_TIM8_CH3         49U  
#define DMA_REQUEST_TIM8_CH4         50U  
#define DMA_REQUEST_TIM8_UP          51U  
#define DMA_REQUEST_TIM8_TRIG        52U  
#define DMA_REQUEST_TIM8_COM         53U  

#define DMA_REQUEST_TIM5_CH1         55U  
#define DMA_REQUEST_TIM5_CH2         56U  
#define DMA_REQUEST_TIM5_CH3         57U  
#define DMA_REQUEST_TIM5_CH4         58U  
#define DMA_REQUEST_TIM5_UP          59U  
#define DMA_REQUEST_TIM5_TRIG        60U  

#define TIM_CHANNEL_1                 0x00000000U
#define TIM_CHANNEL_2                 0x00000004U
#define TIM_CHANNEL_3                 0x00000008U
#define TIM_CHANNEL_4                 0x0000000CU
#define TIM_CHANNEL_5                 0x00000010U
#define TIM_CHANNEL_6                 0x00000014U

#define TIM_AUTORELOAD_PRELOAD_DISABLE                UINT32_MAX
#define TIM_AUTORELOAD_PRELOAD_ENABLE                 (UINT32_MAX - 1U)            

#define TIM_CLOCKDIVISION_DIV1             UINT32_MAX                     
#define TIM_CLOCKDIVISION_DIV2             (UINT32_MAX - 1U)                   
#define TIM_CLOCKDIVISION_DIV4             (UINT32_MAX - 2U) 

#define TIM_COUNTERMODE_UP                 UINT32_MAX                     
#define TIM_COUNTERMODE_DOWN               (UINT32_MAX - 1U)              
#define TIM_COUNTERMODE_CENTERALIGNED1     (UINT32_MAX - 2U)              
#define TIM_COUNTERMODE_CENTERALIGNED2     (UINT32_MAX - 3U)              
#define TIM_COUNTERMODE_CENTERALIGNED3     (UINT32_MAX - 4U)    

#define TIM_OCMODE_TIMING                 0x0000U
#define TIM_OCMODE_ACTIVE                 0x0001U
#define TIM_OCMODE_INACTIVE               0x0002U
#define TIM_OCMODE_TOGGLE                 0x0003U
#define TIM_OCMODE_PWM1                   0x0006U
#define TIM_OCMODE_PWM2                   0x0007U

#define TIM_OCPOLARITY_HIGH               0x0000U
#define TIM_OCPOLARITY_LOW                0x0001U

#define TIM_OCNPOLARITY_HIGH              0x0000U
#define TIM_OCNPOLARITY_LOW               0x0008U

#define TIM_OCFAST_DISABLE                0x0000U
#define TIM_OCFAST_ENABLE                 0x0004U

#define TIM_OCIDLESTATE_SET               0x0100U
#define TIM_OCIDLESTATE_RESET             0x0000U

#define TIM_OCNIDLESTATE_SET              0x0200U
#define TIM_OCNIDLESTATE_RESET            0x0000U

#define TIM_DMA_ID_UPDATE                 0x0000U
#define TIM_DMA_ID_CC1                    0x0001U
#define TIM_DMA_ID_CC2                    0x0002U
#define TIM_DMA_ID_CC3                    0x0003U
#define TIM_DMA_ID_CC4                    0x0004U
#define TIM_DMA_ID_COM                    0x0005U
#define TIM_DMA_ID_TRIGGER                0x0006U

#define __HAL_RCC_TIM5_CLK_ENABLE()
#define __HAL_RCC_TIM8_CLK_ENABLE()
#define __HAL_RCC_TIM12_CLK_ENABLE()
#define __HAL_RCC_TIM13_CLK_ENABLE()

#define __HAL_TIM_SET_PRESCALER(htim, value)     ((htim)->Instance->PSC = (value))
#define __HAL_TIM_SET_AUTORELOAD(htim, value)    ((htim)->Instance->ARR = (value))
#define __HAL_TIM_SET_COMPARE(htim, channel, value) \
    do { \
        switch(channel) { \
            case TIM_CHANNEL_1: (htim)->Instance->CCR1 = (value); break; \
            case TIM_CHANNEL_2: (htim)->Instance->CCR2 = (value); break; \
            case TIM_CHANNEL_3: (htim)->Instance->CCR3 = (value); break; \
            case TIM_CHANNEL_4: (htim)->Instance->CCR4 = (value); break; \
        } \
    } while(0)

#define TIM_CHANNEL_STATE_GET(htim, channel) \
    (((channel) == TIM_CHANNEL_1) ? (htim)->ChannelState[0] : \
    ((channel) == TIM_CHANNEL_2) ? (htim)->ChannelState[1] : \
    ((channel) == TIM_CHANNEL_3) ? (htim)->ChannelState[2] : \
    ((channel) == TIM_CHANNEL_4) ? (htim)->ChannelState[3] : \
    ((channel) == TIM_CHANNEL_5) ? (htim)->ChannelState[4] : \
    ((channel) == TIM_CHANNEL_6) ? (htim)->ChannelState[5] : HAL_TIM_CHANNEL_STATE_RESET)

typedef enum
{
  HAL_TIM_CHANNEL_STATE_RESET             = 0x00U,    /*!< TIM Channel initial state                         */
  HAL_TIM_CHANNEL_STATE_READY             = 0x01U,    /*!< TIM Channel ready for use                         */
  HAL_TIM_CHANNEL_STATE_BUSY              = 0x02U,    /*!< An internal process is ongoing on the TIM channel */
} HAL_TIM_ChannelStateTypeDef;

typedef struct
{
  uint32_t Prescaler;                                       
  uint32_t CounterMode;                                    
  uint32_t Period;                                 
  uint32_t ClockDivision;     
  uint32_t RepetitionCounter;                           
  uint32_t AutoReloadPreload;                             
} TIM_Base_InitTypeDef;

typedef struct
{
  TIM_TypeDef                        *Instance;     
  TIM_Base_InitTypeDef               Init;          
  HAL_TIM_ActiveChannel              Channel;       
  DMA_HandleTypeDef                  *hdma[7];          
  HAL_TIM_ChannelStateTypeDef   ChannelState[6];
  HAL_TIM_ChannelStateTypeDef   ChannelNState[4];
} TIM_HandleTypeDef;

typedef struct
{
  uint32_t OCMode;      
  uint32_t Pulse;       
  uint32_t OCPolarity;  
  uint32_t OCNPolarity; 
  uint32_t OCFastMode;  
  uint32_t OCIdleState; 
  uint32_t OCNIdleState;          
} TIM_OC_InitTypeDef;


HAL_StatusTypeDef HAL_TIM_PWM_Init(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIM_PWM_DeInit(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIM_PWM_ConfigChannel(TIM_HandleTypeDef *htim, TIM_OC_InitTypeDef *sConfig, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_PWM_Stop(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_PWM_Start_DMA(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t *pData, uint16_t Length);
HAL_StatusTypeDef HAL_TIM_PWM_Stop_DMA(TIM_HandleTypeDef *htim, uint32_t Channel);

void HAL_NVIC_SetPriority(uint32_t IRQn, uint32_t priorityGroup, uint32_t subPriority);
void HAL_NVIC_EnableIRQ(uint32_t IRQn);

typedef struct
{
  uint32_t CR1;         
  uint32_t CR2;         
  uint32_t CFG1;        
  uint32_t CFG2;        
  uint32_t IER;         
  uint32_t SR;          
  uint32_t IFCR;        
  uint32_t RESERVED0;   
  uint32_t TXDR;        
  uint32_t RESERVED1[3];
  uint32_t RXDR;        
  uint32_t RESERVED2[3];
  uint32_t CRCPOLY;     
  uint32_t TXCRC;       
  uint32_t RXCRC;       
  uint32_t UDRDR;       
  uint32_t I2SCFGR;
} SPI_TypeDef;

typedef struct
{
  uint32_t Mode;                                                    
  uint32_t Direction;                     
  uint32_t DataSize;                      
  uint32_t CLKPolarity;                   
  uint32_t CLKPhase;                      
  uint32_t NSS;                           
  uint32_t BaudRatePrescaler;             
  uint32_t FirstBit;                      
  uint32_t TIMode;                        
  uint32_t CRCCalculation;                
  uint32_t CRCPolynomial;                 
  uint32_t CRCLength;                     
  uint32_t NSSPMode;                      
  uint32_t NSSPolarity;                   
  uint32_t FifoThreshold;                 
  uint32_t TxCRCInitializationPattern;    
  uint32_t RxCRCInitializationPattern;    
  uint32_t MasterSSIdleness;              
  uint32_t MasterInterDataIdleness;       
  uint32_t MasterReceiverAutoSusp;        
  uint32_t MasterKeepIOState;             
  uint32_t IOSwap;                                              
} SPI_InitTypeDef;

typedef struct __SPI_HandleTypeDef
{
  SPI_TypeDef                *Instance;                
  SPI_InitTypeDef            Init;                     
} SPI_HandleTypeDef;

extern SPI_TypeDef g_SPI1;
extern SPI_TypeDef g_SPI2;
extern SPI_TypeDef g_SPI3;
extern SPI_TypeDef g_SPI4;
extern SPI_TypeDef g_SPI5;

#define SPI1               ((SPI_TypeDef *) &g_SPI1)
#define SPI2               ((SPI_TypeDef *) &g_SPI2)
#define SPI3               ((SPI_TypeDef *) &g_SPI3)
#define SPI4               ((SPI_TypeDef *) &g_SPI4)
#define SPI5               ((SPI_TypeDef *) &g_SPI5)

#define HAL_MAX_DELAY      0xFFFFFFFFU

#define SPI_MODE_SLAVE                  0x00000000U
#define SPI_MODE_MASTER                 0x00400000U

#define SPI_DIRECTION_2LINES            0x00000000U
#define SPI_DIRECTION_2LINES_RXONLY     0x00000400U
#define SPI_DIRECTION_1LINE             0x00008000U
#define SPI_DIRECTION_1LINE_RX          0x0000C000U

#define SPI_DATASIZE_4BIT               0x00000300U
#define SPI_DATASIZE_5BIT               0x00000400U
#define SPI_DATASIZE_6BIT               0x00000500U
#define SPI_DATASIZE_7BIT               0x00000600U
#define SPI_DATASIZE_8BIT               0x00000700U
#define SPI_DATASIZE_9BIT               0x00000800U
#define SPI_DATASIZE_10BIT              0x00000900U
#define SPI_DATASIZE_11BIT              0x00000A00U
#define SPI_DATASIZE_12BIT              0x00000B00U
#define SPI_DATASIZE_13BIT              0x00000C00U
#define SPI_DATASIZE_14BIT              0x00000D00U
#define SPI_DATASIZE_15BIT              0x00000E00U
#define SPI_DATASIZE_16BIT              0x00000F00U

#define SPI_POLARITY_LOW                0x00000000U
#define SPI_POLARITY_HIGH               0x00000002U

#define SPI_PHASE_1EDGE                 0x00000000U
#define SPI_PHASE_2EDGE                 0x00000001U

#define SPI_NSS_SOFT                    0x00000200U
#define SPI_NSS_HARD_INPUT              0x00000000U
#define SPI_NSS_HARD_OUTPUT             0x00000004U

#define SPI_NSS_PULSE_ENABLE            0x00000000U
#define SPI_NSS_PULSE_DISABLE           0x00000008U

#define SPI_NSS_POLARITY_LOW            0x00000000U
#define SPI_NSS_POLARITY_HIGH           0x00000001U

#define SPI_BAUDRATEPRESCALER_2         0x00000000U
#define SPI_BAUDRATEPRESCALER_4         0x00000008U
#define SPI_BAUDRATEPRESCALER_8         0x00000010U
#define SPI_BAUDRATEPRESCALER_16        0x00000018U
#define SPI_BAUDRATEPRESCALER_32        0x00000020U
#define SPI_BAUDRATEPRESCALER_64        0x00000028U
#define SPI_BAUDRATEPRESCALER_128       0x00000030U
#define SPI_BAUDRATEPRESCALER_256       0x00000038U

#define SPI_FIRSTBIT_MSB                0x00000000U
#define SPI_FIRSTBIT_LSB                0x00000080U

#define SPI_TIMODE_DISABLE              0x00000000U
#define SPI_TIMODE_ENABLE               0x00000001U

#define SPI_CRCCALCULATION_DISABLE      0x00000000U
#define SPI_CRCCALCULATION_ENABLE       0x00002000U

#define SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN    0x00000000U
#define SPI_CRC_INITIALIZATION_ALL_ONE_PATTERN     0x00000001U

#define SPI_FIFO_THRESHOLD_01DATA       0x00000000U
#define SPI_FIFO_THRESHOLD_02DATA       0x00000020U
#define SPI_FIFO_THRESHOLD_03DATA       0x00000040U
#define SPI_FIFO_THRESHOLD_04DATA       0x00000060U
#define SPI_FIFO_THRESHOLD_05DATA       0x00000080U
#define SPI_FIFO_THRESHOLD_06DATA       0x000000A0U
#define SPI_FIFO_THRESHOLD_07DATA       0x000000C0U
#define SPI_FIFO_THRESHOLD_08DATA       0x000000E0U

#define SPI_MASTER_SS_IDLENESS_00CYCLE  0x00000000U
#define SPI_MASTER_SS_IDLENESS_01CYCLE  0x00000001U
#define SPI_MASTER_SS_IDLENESS_02CYCLE  0x00000002U
#define SPI_MASTER_SS_IDLENESS_03CYCLE  0x00000003U
#define SPI_MASTER_SS_IDLENESS_04CYCLE  0x00000004U
#define SPI_MASTER_SS_IDLENESS_05CYCLE  0x00000005U
#define SPI_MASTER_SS_IDLENESS_06CYCLE  0x00000006U
#define SPI_MASTER_SS_IDLENESS_07CYCLE  0x00000007U
#define SPI_MASTER_SS_IDLENESS_08CYCLE  0x00000008U
#define SPI_MASTER_SS_IDLENESS_09CYCLE  0x00000009U
#define SPI_MASTER_SS_IDLENESS_10CYCLE  0x0000000AU
#define SPI_MASTER_SS_IDLENESS_11CYCLE  0x0000000BU
#define SPI_MASTER_SS_IDLENESS_12CYCLE  0x0000000CU
#define SPI_MASTER_SS_IDLENESS_13CYCLE  0x0000000DU
#define SPI_MASTER_SS_IDLENESS_14CYCLE  0x0000000EU
#define SPI_MASTER_SS_IDLENESS_15CYCLE  0x0000000FU

#define SPI_MASTER_INTERDATA_IDLENESS_00CYCLE    0x00000000U
#define SPI_MASTER_INTERDATA_IDLENESS_01CYCLE    0x00000010U
#define SPI_MASTER_INTERDATA_IDLENESS_02CYCLE    0x00000020U
#define SPI_MASTER_INTERDATA_IDLENESS_03CYCLE    0x00000030U
#define SPI_MASTER_INTERDATA_IDLENESS_04CYCLE    0x00000040U
#define SPI_MASTER_INTERDATA_IDLENESS_05CYCLE    0x00000050U
#define SPI_MASTER_INTERDATA_IDLENESS_06CYCLE    0x00000060U
#define SPI_MASTER_INTERDATA_IDLENESS_07CYCLE    0x00000070U
#define SPI_MASTER_INTERDATA_IDLENESS_08CYCLE    0x00000080U
#define SPI_MASTER_INTERDATA_IDLENESS_09CYCLE    0x00000090U
#define SPI_MASTER_INTERDATA_IDLENESS_10CYCLE    0x000000A0U
#define SPI_MASTER_INTERDATA_IDLENESS_11CYCLE    0x000000B0U
#define SPI_MASTER_INTERDATA_IDLENESS_12CYCLE    0x000000C0U
#define SPI_MASTER_INTERDATA_IDLENESS_13CYCLE    0x000000D0U
#define SPI_MASTER_INTERDATA_IDLENESS_14CYCLE    0x000000E0U
#define SPI_MASTER_INTERDATA_IDLENESS_15CYCLE    0x000000F0U

#define SPI_MASTER_RX_AUTOSUSP_DISABLE  0x00000000U
#define SPI_MASTER_RX_AUTOSUSP_ENABLE   0x00008000U

#define SPI_MASTER_KEEP_IO_STATE_DISABLE    0x00000000U
#define SPI_MASTER_KEEP_IO_STATE_ENABLE     0x00004000U

#define SPI_IO_SWAP_DISABLE             0x00000000U
#define SPI_IO_SWAP_ENABLE              0x00008000U

#define RCC_PERIPHCLK_SPI1              0x00000001U
#define RCC_PERIPHCLK_SPI2              0x00000002U
#define RCC_PERIPHCLK_SPI3              0x00000004U
#define RCC_PERIPHCLK_SPI4              0x00000008U
#define RCC_PERIPHCLK_SPI5              0x00000010U

#define RCC_SPI123CLKSOURCE_PLL         0x00000000U
#define RCC_SPI123CLKSOURCE_PLL2        0x00000001U
#define RCC_SPI123CLKSOURCE_PLL3        0x00000002U
#define RCC_SPI123CLKSOURCE_I2S_CKIN    0x00000003U
#define RCC_SPI123CLKSOURCE_CLKP        0x00000004U

#define RCC_SPI45CLKSOURCE_PCLK2        0x00000000U
#define RCC_SPI45CLKSOURCE_PLL2         0x00000001U
#define RCC_SPI45CLKSOURCE_PLL3         0x00000002U
#define RCC_SPI45CLKSOURCE_HSI          0x00000003U
#define RCC_SPI45CLKSOURCE_CSI          0x00000004U
#define RCC_SPI45CLKSOURCE_HSE          0x00000005U

#define __HAL_RCC_SPI1_CLK_ENABLE()
#define __HAL_RCC_SPI2_CLK_ENABLE()
#define __HAL_RCC_SPI3_CLK_ENABLE()
#define __HAL_RCC_SPI4_CLK_ENABLE()
#define __HAL_RCC_SPI5_CLK_ENABLE()

typedef struct
{
  uint32_t PLL2M;     
  uint32_t PLL2N;                 
  uint32_t PLL2P;     
  uint32_t PLL2Q;     
  uint32_t PLL2R;                
  uint32_t PLL2RGE;               
  uint32_t PLL2VCOSEL;
  uint32_t PLL2FRACN;             
} RCC_PLL2InitTypeDef;

typedef struct
{
  uint32_t PLL3M;     
  uint32_t PLL3N;     
  uint32_t PLL3P;     
  uint32_t PLL3Q;     
  uint32_t PLL3R;               
  uint32_t PLL3RGE;              
  uint32_t PLL3VCOSEL;
  uint32_t PLL3FRACN;              
} RCC_PLL3InitTypeDef;

typedef struct
{
  uint64_t PeriphClockSelection;
  RCC_PLL2InitTypeDef PLL2;
  RCC_PLL3InitTypeDef PLL3;
  uint32_t FmcClockSelection;
  uint32_t QspiClockSelection;
  uint32_t OspiClockSelection;
  uint32_t DsiClockSelection;
  uint32_t SdmmcClockSelection;
  uint32_t CkperClockSelection;
  uint32_t Sai1ClockSelection;
  uint32_t Sai23ClockSelection;
  uint32_t Sai2AClockSelection;
  uint32_t Sai2BClockSelection;
  uint32_t Spi123ClockSelection;
  uint32_t Spi45ClockSelection;
  uint32_t SpdifrxClockSelection;
  uint32_t Dfsdm1ClockSelection;
  uint32_t Dfsdm2ClockSelection;
  uint32_t FdcanClockSelection;
  uint32_t Swpmi1ClockSelection;
  uint32_t Usart234578ClockSelection;
  uint32_t Usart16ClockSelection;
  uint32_t RngClockSelection;
  uint32_t I2c1235ClockSelection;
  uint32_t UsbClockSelection;
  uint32_t CecClockSelection;
  uint32_t Lptim1ClockSelection;
  uint32_t Lpuart1ClockSelection;
  uint32_t I2c4ClockSelection;
  uint32_t Lptim2ClockSelection;
  uint32_t Lptim345ClockSelection;
  uint32_t AdcClockSelection;
  uint32_t Sai4AClockSelection;
  uint32_t Sai4BClockSelection;
  uint32_t Spi6ClockSelection;
  uint32_t RTCClockSelection;
  uint32_t Hrtim1ClockSelection;
  uint32_t TIMPresSelection;
} RCC_PeriphCLKInitTypeDef;

HAL_StatusTypeDef HAL_SPI_Init(SPI_HandleTypeDef *hspi);
HAL_StatusTypeDef HAL_SPI_DeInit(SPI_HandleTypeDef *hspi);
HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_SPI_Receive(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_RCCEx_PeriphCLKConfig(RCC_PeriphCLKInitTypeDef *PeriphClkInit);

// UART Word Length definitions
#define UART_WORDLENGTH_7B              0x10000000U
#define UART_WORDLENGTH_8B              0x00000000U
#define UART_WORDLENGTH_9B              0x00001000U

// UART Stop Bits definitions
#define UART_STOPBITS_0_5               0x00001000U
#define UART_STOPBITS_1                 0x00000000U
#define UART_STOPBITS_1_5               0x00003000U
#define UART_STOPBITS_2                 0x00002000U

// UART Parity definitions
#define UART_PARITY_NONE                0x00000000U
#define UART_PARITY_EVEN                0x00000400U
#define UART_PARITY_ODD                 0x00000600U

// UART Mode definitions
#define UART_MODE_RX                    0x00000004U
#define UART_MODE_TX                    0x00000008U
#define UART_MODE_TX_RX                 0x0000000CU

// UART Hardware Flow Control definitions
#define UART_HWCONTROL_NONE             0x00000000U
#define UART_HWCONTROL_RTS              0x00000100U
#define UART_HWCONTROL_CTS              0x00000200U
#define UART_HWCONTROL_RTS_CTS          0x00000300U

// UART Over Sampling definitions
#define UART_OVERSAMPLING_16            0x00000000U
#define UART_OVERSAMPLING_8             0x00008000U

// UART One Bit Sampling definitions
#define UART_ONE_BIT_SAMPLE_DISABLE     0x00000000U
#define UART_ONE_BIT_SAMPLE_ENABLE      0x00000800U

// UART Clock Prescaler definitions
#define UART_PRESCALER_DIV1             0x00000000U
#define UART_PRESCALER_DIV2             0x00000001U
#define UART_PRESCALER_DIV4             0x00000002U
#define UART_PRESCALER_DIV6             0x00000003U
#define UART_PRESCALER_DIV8             0x00000004U
#define UART_PRESCALER_DIV10            0x00000005U
#define UART_PRESCALER_DIV12            0x00000006U
#define UART_PRESCALER_DIV16            0x00000007U
#define UART_PRESCALER_DIV32            0x00000008U
#define UART_PRESCALER_DIV64            0x00000009U
#define UART_PRESCALER_DIV128           0x0000000AU
#define UART_PRESCALER_DIV256           0x0000000BU

// UART Advanced Feature Initialization definitions
#define UART_ADVFEATURE_NO_INIT                 0x00000000U
#define UART_ADVFEATURE_TXINVERT_INIT           0x00000001U
#define UART_ADVFEATURE_RXINVERT_INIT           0x00000002U
#define UART_ADVFEATURE_DATAINVERT_INIT         0x00000004U
#define UART_ADVFEATURE_SWAP_INIT               0x00000008U
#define UART_ADVFEATURE_RXOVERRUNDISABLE_INIT   0x00000010U
#define UART_ADVFEATURE_DMADISABLEONERROR_INIT  0x00000020U
#define UART_ADVFEATURE_AUTOBAUDRATE_INIT       0x00000040U
#define UART_ADVFEATURE_MSBFIRST_INIT           0x00000080U

// UART FIFO Threshold definitions
#define UART_TXFIFO_THRESHOLD_1_8       0x00000000U
#define UART_TXFIFO_THRESHOLD_1_4       0x00000001U
#define UART_TXFIFO_THRESHOLD_1_2       0x00000002U
#define UART_TXFIFO_THRESHOLD_3_4       0x00000003U
#define UART_TXFIFO_THRESHOLD_7_8       0x00000004U
#define UART_TXFIFO_THRESHOLD_EMPTY     0x00000005U

#define UART_RXFIFO_THRESHOLD_1_8       0x00000000U
#define UART_RXFIFO_THRESHOLD_1_4       0x00000001U
#define UART_RXFIFO_THRESHOLD_1_2       0x00000002U
#define UART_RXFIFO_THRESHOLD_3_4       0x00000003U
#define UART_RXFIFO_THRESHOLD_7_8       0x00000004U
#define UART_RXFIFO_THRESHOLD_FULL      0x00000005U

// UART Interrupt definitions
#define UART_IT_PE                          0x0028U              /*!< UART parity error interruption                 */
#define UART_IT_TXE                         0x0727U              /*!< UART transmit data register empty interruption */
#define UART_IT_TXFNF                       0x0727U              /*!< UART TX FIFO not full interruption             */
#define UART_IT_TC                          0x0626U              /*!< UART transmission complete interruption        */
#define UART_IT_RXNE                        0x0525U              /*!< UART read data register not empty interruption */
#define UART_IT_RXFNE                       0x0525U              /*!< UART RXFIFO not empty interruption             */
#define UART_IT_IDLE                        0x0424U              /*!< UART idle interruption                         */
#define UART_IT_LBD                         0x0846U              /*!< UART LIN break detection interruption          */
#define UART_IT_CTS                         0x096AU              /*!< UART CTS interruption                          */
#define UART_IT_CM                          0x112EU              /*!< UART character match interruption              */
#define UART_IT_WUF                         0x1476U              /*!< UART wake-up from stop mode interruption       */
#define UART_IT_RXFF                        0x183FU              /*!< UART RXFIFO full interruption                  */
#define UART_IT_TXFE                        0x173EU              /*!< UART TXFIFO empty interruption                 */
#define UART_IT_RXFT                        0x1A7CU              /*!< UART RXFIFO threshold reached interruption     */
#define UART_IT_TXFT                        0x1B77U              /*!< UART TXFIFO threshold reached interruption     */
#define UART_IT_RTO                         0x0B3AU              /*!< UART receiver timeout interruption             */
#define UART_IT_ERR                         0x0060U              /*!< UART error interruption                        */
#define UART_IT_ORE                         0x0300U              /*!< UART overrun error interruption                */
#define UART_IT_NE                          0x0200U              /*!< UART noise error interruption                  */
#define UART_IT_FE                          0x0100U              /*!< UART frame error interruption                  */

// RCC UART/USART Clock Source definitions
#define RCC_PERIPHCLK_USART1            0x00000001U
#define RCC_PERIPHCLK_USART2            0x00000002U
#define RCC_PERIPHCLK_USART3            0x00000004U
#define RCC_PERIPHCLK_UART4             0x00000008U
#define RCC_PERIPHCLK_UART5             0x00000010U
#define RCC_PERIPHCLK_USART6            0x00000020U

#define RCC_USART16CLKSOURCE_D2PCLK2    0x00000000U
#define RCC_USART16CLKSOURCE_PLL2       0x00000001U
#define RCC_USART16CLKSOURCE_PLL3       0x00000002U
#define RCC_USART16CLKSOURCE_HSI        0x00000003U
#define RCC_USART16CLKSOURCE_CSI        0x00000004U
#define RCC_USART16CLKSOURCE_LSE        0x00000005U

#define RCC_USART234578CLKSOURCE_D2PCLK1    0x00000000U
#define RCC_USART234578CLKSOURCE_PLL2       0x00000001U
#define RCC_USART234578CLKSOURCE_PLL3       0x00000002U
#define RCC_USART234578CLKSOURCE_HSI        0x00000003U
#define RCC_USART234578CLKSOURCE_CSI        0x00000004U
#define RCC_USART234578CLKSOURCE_LSE        0x00000005U

// IRQ Numbers
typedef enum {
    USART1_IRQn     = 37,
    USART2_IRQn     = 38,
    USART3_IRQn     = 39,
    UART4_IRQn      = 52,
    UART5_IRQn      = 53,
    USART6_IRQn     = 71,
} IRQn_Type;

// RCC Clock Enable Macros
#define __HAL_RCC_USART1_CLK_ENABLE()
#define __HAL_RCC_USART2_CLK_ENABLE()
#define __HAL_RCC_USART3_CLK_ENABLE()
#define __HAL_RCC_UART4_CLK_ENABLE() 
#define __HAL_RCC_UART5_CLK_ENABLE() 
#define __HAL_RCC_USART6_CLK_ENABLE()

// UART IT Enable/Disable Macros
#define __HAL_UART_ENABLE_IT(huart, interrupt) 
#define __HAL_UART_DISABLE_IT(huart, interrupt)

// UART Type definitions
typedef struct {
    uint32_t BaudRate;
    uint32_t WordLength;
    uint32_t StopBits;
    uint32_t Parity;
    uint32_t Mode;
    uint32_t HwFlowCtl;
    uint32_t OverSampling;
    uint32_t OneBitSampling;
    uint32_t ClockPrescaler;
} UART_InitTypeDef;

typedef struct {
    uint32_t AdvFeatureInit;
    // Add other advanced init fields as needed
} UART_AdvFeatureInitTypeDef;

typedef struct
{
 uint32_t CR1;  
 uint32_t CR2;  
 uint32_t CR3;  
 uint32_t BRR;  
 uint32_t GTPR; 
 uint32_t RTOR; 
 uint32_t RQR;  
 uint32_t ISR;  
 uint32_t ICR;  
 uint32_t RDR;  
 uint32_t TDR;  
 uint32_t PRESC;
} USART_TypeDef;

typedef struct {
    USART_TypeDef *Instance;
    UART_InitTypeDef Init;
    UART_AdvFeatureInitTypeDef AdvancedInit;
    // Add other handle fields as needed
} UART_HandleTypeDef;

// Mock UART instances
extern USART_TypeDef g_USART1;
extern USART_TypeDef g_USART2;
extern USART_TypeDef g_USART3;
extern USART_TypeDef g_UART4;
extern USART_TypeDef g_UART5;
extern USART_TypeDef g_USART6;

#define USART1              ((USART_TypeDef *) &g_USART1)
#define USART2              ((USART_TypeDef *) &g_USART2)
#define USART3              ((USART_TypeDef *) &g_USART3)
#define UART4               ((USART_TypeDef *) &g_UART4)
#define UART5               ((USART_TypeDef *) &g_UART5)
#define USART6              ((USART_TypeDef *) &g_USART6)

// UART Function declarations
HAL_StatusTypeDef HAL_UART_Init(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UART_DeInit(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *huart, const uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_UART_Receive(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
void HAL_UART_IRQHandler(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UARTEx_SetTxFifoThreshold(UART_HandleTypeDef *huart, uint32_t Threshold);
HAL_StatusTypeDef HAL_UARTEx_SetRxFifoThreshold(UART_HandleTypeDef *huart, uint32_t Threshold);
HAL_StatusTypeDef HAL_UARTEx_DisableFifoMode(UART_HandleTypeDef *huart);

typedef HAL_StatusTypeDef (*HAL_SPI_Transmit_CB) (SPI_HandleTypeDef* hspi, uint8_t* pData, uint16_t Size, uint32_t Timeout);
typedef HAL_StatusTypeDef (*HAL_SPI_TransmitReceive_CB) (SPI_HandleTypeDef* hspi, uint8_t* pTxData, uint8_t* pRxData, uint16_t Size, uint32_t Timeout);
typedef HAL_StatusTypeDef (*HAL_UART_Transmit_CB) (UART_HandleTypeDef* huart, const uint8_t* pData, uint16_t Size, uint32_t Timeout);

// Function pointer declarations for mocking
extern void (*HAL_Delay_Mock)(uint32_t ms);
extern uint32_t (*HAL_GetTick_Mock)(void);
extern uint32_t (*HAL_GetCurrentCPUID_Mock)(void);
extern void (*__disable_irq_Mock)(void);

// Timer function pointers
extern HAL_StatusTypeDef (*HAL_TIM_PWM_Init_Mock)(TIM_HandleTypeDef* htim);
extern HAL_StatusTypeDef (*HAL_TIM_PWM_ConfigChannel_Mock)(TIM_HandleTypeDef* htim, TIM_OC_InitTypeDef* sConfig, uint32_t Channel);
extern HAL_StatusTypeDef (*HAL_TIM_PWM_Start_Mock)(TIM_HandleTypeDef* htim, uint32_t Channel);
extern HAL_StatusTypeDef (*HAL_TIM_PWM_Stop_Mock)(TIM_HandleTypeDef* htim, uint32_t Channel);
extern HAL_StatusTypeDef (*HAL_TIM_PWM_Start_DMA_Mock)(TIM_HandleTypeDef* htim, uint32_t Channel, uint32_t* pData, uint16_t Length);
extern HAL_StatusTypeDef (*HAL_TIM_PWM_Stop_DMA_Mock)(TIM_HandleTypeDef* htim, uint32_t Channel);
extern void (*HAL_TIM_ErrorCallback_Mock)(TIM_HandleTypeDef* htim);
extern void (*HAL_TIM_PWM_PulseFinishedCallback_Mock)(TIM_HandleTypeDef* htim);
extern void (*HAL_TIM_PWM_PulseFinishedHalfCpltCallback_Mock)(TIM_HandleTypeDef* htim);

// SPI function pointers
extern HAL_StatusTypeDef (*HAL_SPI_Init_Mock)(SPI_HandleTypeDef* hspi);
extern HAL_StatusTypeDef (*HAL_SPI_DeInit_Mock)(SPI_HandleTypeDef* hspi);
extern HAL_StatusTypeDef (*HAL_SPI_Transmit_Mock)(SPI_HandleTypeDef* hspi, uint8_t* pData, uint16_t Size, uint32_t Timeout);
extern HAL_StatusTypeDef (*HAL_SPI_Receive_Mock)(SPI_HandleTypeDef* hspi, uint8_t* pData, uint16_t Size, uint32_t Timeout);
extern HAL_StatusTypeDef (*HAL_SPI_TransmitReceive_Mock)(SPI_HandleTypeDef* hspi, uint8_t* pTxData, uint8_t* pRxData, uint16_t Size, uint32_t Timeout);

// UART function pointers
extern HAL_StatusTypeDef (*HAL_UART_Init_Mock)(UART_HandleTypeDef* huart);
extern HAL_StatusTypeDef (*HAL_UART_DeInit_Mock)(UART_HandleTypeDef* huart);
extern HAL_StatusTypeDef (*HAL_UART_Transmit_Mock)(UART_HandleTypeDef* huart, const uint8_t* pData, uint16_t Size, uint32_t Timeout);
extern HAL_StatusTypeDef (*HAL_UART_Receive_Mock)(UART_HandleTypeDef* huart, uint8_t* pData, uint16_t Size, uint32_t Timeout);
extern HAL_StatusTypeDef (*HAL_UART_Receive_IT_Mock)(UART_HandleTypeDef* huart, uint8_t* pData, uint16_t Size);
extern void (*HAL_UART_IRQHandler_Mock)(UART_HandleTypeDef* huart);
extern HAL_StatusTypeDef (*HAL_UARTEx_SetTxFifoThreshold_Mock)(UART_HandleTypeDef* huart, uint32_t Threshold);
extern HAL_StatusTypeDef (*HAL_UARTEx_SetRxFifoThreshold_Mock)(UART_HandleTypeDef* huart, uint32_t Threshold);
extern HAL_StatusTypeDef (*HAL_UARTEx_DisableFifoMode_Mock)(UART_HandleTypeDef* huart);
extern void (*HAL_UART_TxCpltCallback_Mock)(UART_HandleTypeDef* huart);
extern void (*HAL_UART_RxCpltCallback_Mock)(UART_HandleTypeDef* huart);
extern void (*HAL_UART_ErrorCallback_Mock)(UART_HandleTypeDef* huart);

// DMA function pointers
extern HAL_StatusTypeDef (*HAL_DMA_Init_Mock)(DMA_HandleTypeDef* hdma);
extern HAL_StatusTypeDef (*HAL_DMA_DeInit_Mock)(DMA_HandleTypeDef* hdma);
extern void (*HAL_DMA_IRQHandler_Mock)(DMA_HandleTypeDef* hdma);
extern HAL_StatusTypeDef (*HAL_DMA_Start_Mock)(DMA_HandleTypeDef* hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength);
extern HAL_StatusTypeDef (*HAL_DMA_Start_IT_Mock)(DMA_HandleTypeDef* hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength);
extern HAL_StatusTypeDef (*HAL_DMA_Abort_Mock)(DMA_HandleTypeDef* hdma);
extern uint32_t (*HAL_DMA_GetError_Mock)(const DMA_HandleTypeDef* hdma);

// RCC function pointers
extern HAL_StatusTypeDef (*HAL_RCCEx_PeriphCLKConfig_Mock)(RCC_PeriphCLKInitTypeDef* PeriphClkInit);

// GPIO function pointers
extern void (*HAL_GPIO_WritePin_Mock)(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint32_t PinState);

// NVIC function pointers
extern void (*HAL_NVIC_SetPriority_Mock)(IRQn_Type IRQn, uint32_t PreemptPriority, uint32_t SubPriority);
extern void (*HAL_NVIC_EnableIRQ_Mock)(IRQn_Type IRQn);
extern void (*HAL_NVIC_DisableIRQ_Mock)(IRQn_Type IRQn);


// clang-format on
// #ifdef UNIT_TEST


#endif // HAL_H

#endif