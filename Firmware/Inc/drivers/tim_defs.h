#ifndef DRIVERS_TIM_DEFS_H
#define DRIVERS_TIM_DEFS_H

#include <stdbool.h>
#include <stdint.h>

#include "common.h"

#include "drivers/io/gpio_defs.h"

#define ONE_MHZ 1000000U

typedef uint8_t eTIM_CHAN_ID_t;
enum {
    eTIM_CHANNEL_NULL_ID = 0,
    eTIM_CHANNEL_1_ID,
    eTIM_CHANNEL_2_ID,
    eTIM_CHANNEL_3_ID,
    eTIM_CHANNEL_4_ID


};

#define TIM_CHAN_ID_TO_INDEX(CHAN_ID)    ((CHAN_ID) - 1U)
#define TIM_CHAN_INDEX_TO_ID(CHAN_INDEX) ((CHAN_INDEX) + 1U)

typedef uint8_t eTIM_DEVICE_ID_t;
enum {
    eTIM_NULL_DEVICE_ID = 0,
    eTIM_1_DEVICE_ID,  // = 8,
    eTIM_2_DEVICE_ID,  // + 8,
    eTIM_3_DEVICE_ID,  // + 8,
    eTIM_4_DEVICE_ID,  // + 8,
    eTIM_5_DEVICE_ID,  // + 8,
    eTIM_8_DEVICE_ID,  // + 8,
    eTIM_9_DEVICE_ID,  // + 8,
    eTIM_10_DEVICE_ID, // + 8,
    eTIM_11_DEVICE_ID, //  + 8,
    eTIM_12_DEVICE_ID, //  + 8,
    eTIM_13_DEVICE_ID, //  + 8,
    eTIM_14_DEVICE_ID, //  + 8
    eTIM_15_DEVICE_ID, //  + 8
    eTIM_16_DEVICE_ID  //  + 8
};

// #define TIM_DEV_ID_TO_INDEX(DEV_ID)     (((DEV_ID) / 8U) - 1U)
// #define TIM_DEV_ID_TO_INDEX(DEV_ID) ((DEV_ID) - 1U)

#define TIM_ID_MAKE(DEV_ID, CHAN_ID)    (eTIM_##DEV_ID##_DEVICE_ID | eTIM_CHANNEL_##CHAN_ID##_ID)
#define TIM_ID_TO_DEVICE_INDEX(TIM_ID)  (TIM_DEV_ID_TO_INDEX ((TIM_ID) & ~0b111))
#define TIM_ID_TO_CHANNEL_INDEX(TIM_ID) (TIM_CHAN_ID_TO_INDEX ((TIM_ID) & 0b111))
#define TIM_ID_TO_DEVICE_ID(TIM_ID)     (((TIM_ID) & ~0b111))
#define TIM_ID_TO_CHANNEL_ID(TIM_ID)    (((TIM_ID) & 0b111))

#define TIM_CHANNEL_GPIO_IDS(TIM_NUM)                     \
    { GPIO_ID_MAKE_OR_INVALID (TARG_TIM_##TIM_NUM##_CH1), \
      GPIO_ID_MAKE_OR_INVALID (TARG_TIM_##TIM_NUM##_CH2), \
      GPIO_ID_MAKE_OR_INVALID (TARG_TIM_##TIM_NUM##_CH3), \
      GPIO_ID_MAKE_OR_INVALID (TARG_TIM_##TIM_NUM##_CH4) }

typedef uint8_t eTIM_INTERRUPT_FLAG_t;
enum {
    eTIM_INTERRUPT_FLAG_CC1 = 0,
    eTIM_INTERRUPT_FLAG_CC2,
    eTIM_INTERRUPT_FLAG_CC3,
    eTIM_INTERRUPT_FLAG_CC4,
    eTIM_INTERRUPT_FLAG_UPDATE,
    eTIM_INTERRUPT_FLAG_TRIGGER,
    eTIM_INTERRUPT_FLAG_BREAK,
    eTIM_INTERRUPT_FLAG_COM
};

typedef uint8_t eTIM_MODE_t;
enum {
    eTIM_MODE_NULL = 0,
    eTIM_MODE_BASE,
    eTIM_MODE_PWM,
    eTIM_MODE_OC,
    eTIM_MODE_IC,
    eTIM_MODE_ONE_PULSE,
    eTIM_MODE_ENCODER
};

typedef struct TimChanCfg_s {

    eGPIO_ID_t gpioId;

} TimChanCfg_t;

typedef struct TimDevCfg_s {

    eTIM_DEVICE_ID_t devId;
    eTIM_MODE_t mode;
    uint8_t irqPrior;
    TimChanCfg_t chanCfg;

} TimDevCfg_t;


typedef struct TimHwCfg_s {

    eTIM_DEVICE_ID_t devId;
    TIM_TypeDef* pInstance;
    uint32_t volatile* pRccEnableReg;
    uint32_t rccEnableMsk;
    uint8_t gpioAf;
    uint8_t irqNum;
    eGPIO_ID_t channelGpioIds[4U];

} TimHwCfg_t;

// typedef struct TimDmaReqMap_s {
//     eTIM_DEVICE_ID_t id;
//     uint8_t cc1;
//     uint8_t cc2;
//     uint8_t cc3;
//     uint8_t cc4;
//     uint8_t update;
// } TimDmaReqMap_t;

typedef struct TimDevice_s {

    TIM_HandleTypeDef handle;
    eTIM_DEVICE_ID_t devId;
    eTIM_MODE_t mode;
    uint8_t irqNum;
    uint8_t irqPrior;

} TimDevice_t;

typedef struct TimChannel_s {

    eTIM_DEVICE_ID_t devId;
    eTIM_CHAN_ID_t chanId;
    TimDevice_t* pTimDev;

} TimChannel_t;

#endif // DRIVERS_TIM_DEFS_H