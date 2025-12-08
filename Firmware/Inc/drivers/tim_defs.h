#ifndef DRIVERS_TIM_DEFS_H
#define DRIVERS_TIM_DEFS_H

#include <stdbool.h>
#include <stdint.h>

#include "common.h"

#include "drivers/io/gpio_defs.h"

typedef uint8_t eTIM_CHAN_ID_t;
enum {
    eTIM_CHANNEL_NULL_ID = 0,
    eTIM_CHANNEL_1_ID,
    eTIM_CHANNEL_2_ID,
    eTIM_CHANNEL_3_ID,
    eTIM_CHANNEL_4_ID


};

#define TIM_CHAN_ID_TO_INDEX(CHAN_ID) ((CHAN_ID) - 1U)

typedef uint8_t eTIM_DEVICE_ID_t;
enum {
    eTIM_NULL_DEVICE_ID = 0,
    eTIM_1_DEVICE_ID    = 8,
    eTIM_2_DEVICE_ID    = eTIM_1_DEVICE_ID + 8,
    eTIM_3_DEVICE_ID    = eTIM_2_DEVICE_ID + 8,
    eTIM_4_DEVICE_ID    = eTIM_3_DEVICE_ID + 8,
    eTIM_5_DEVICE_ID    = eTIM_4_DEVICE_ID + 8,
    eTIM_8_DEVICE_ID    = eTIM_5_DEVICE_ID + 8,
    eTIM_9_DEVICE_ID    = eTIM_8_DEVICE_ID + 8,
    eTIM_10_DEVICE_ID   = eTIM_9_DEVICE_ID + 8,
    eTIM_11_DEVICE_ID   = eTIM_10_DEVICE_ID + 8,
    eTIM_12_DEVICE_ID   = eTIM_11_DEVICE_ID + 8,
    eTIM_13_DEVICE_ID   = eTIM_12_DEVICE_ID + 8,
    eTIM_14_DEVICE_ID   = eTIM_13_DEVICE_ID + 8
};

#define TIM_DEV_ID_TO_INDEX(DEV_ID) (((DEV_ID) / 8U) - 1U)

typedef uint8_t eTIM_ID_t;

#define TIM_ID_MAKE(DEV_ID, CHAN_ID)    (eTIM_##DEV_ID##_DEVICE_ID | eTIM_CHANNEL_##CHAN_ID##_ID)
#define TIM_ID_TO_DEVICE_INDEX(TIM_ID)  (TIM_DEV_ID_TO_INDEX ((TIM_ID) & ~0b111))
#define TIM_ID_TO_CHANNEL_INDEX(TIM_ID) (TIM_CHAN_ID_TO_INDEX ((TIM_ID) & 0b111))
#define TIM_ID_TO_DEVICE_ID(TIM_ID)     (((TIM_ID) & ~0b111))
#define TIM_ID_TO_CHANNEL_ID(TIM_ID)    (((TIM_ID) & 0b111))

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

typedef uint8_t eTIM_MODE_TYPE_t;
enum {
    eTIM_MODE_NULL = 0,
    eTIM_MODE_PWM,
    eTIM_MODE_OC,
    eTIM_MODE_IC,
    eTIM_MODE_ONE_PULSE,
    eTIM_MODE_ENCODER
};

typedef struct TimCfg_s {
    eTIM_ID_t id;
    eGPIO_ID_t gpioId;
    eTIM_MODE_TYPE_t modeType;
    uint8_t irqPriority;
} TimCfg_t;

typedef struct TimHwCfg_s {
    TIM_TypeDef* pInstance;
    volatile uint32_t* pRccEnableReg;
    uint32_t rccEnableMsk;
    uint8_t gpioAf;
    uint8_t irqNum;
} TimHwCfg_t;

typedef struct TimBaseDevice_s {
    TIM_HandleTypeDef handle;
} TimBaseDevice_t;

typedef struct TimDevice_s {
    eTIM_ID_t id;
    eTIM_MODE_TYPE_t modeType;
    uint8_t irqNum;
    uint8_t irqPriority;
    TimBaseDevice_t* pTimBaseDev;
} TimDevice_t;

#endif // DRIVERS_TIM_DEFS_H