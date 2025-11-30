#ifndef CONF_BASE_H
#define CONF_BASE_H

#include "common.h"
#include "conf/ids.h"
#include <stdbool.h>
#include <stdint.h>

// typedef struct {
//     uint32_t mode;
//     uint32_t pull;
//     uint32_t speed;
//     uint8_t alternate;
// } GPIOSharedConf_t;

typedef struct {
    eGPIO_ID_t id;
    // GPIOSharedConf_t const* pShared;
    // GPIOSharedConf_t conf;
    uint32_t mode;
    uint32_t pull;
    uint32_t speed;
    uint8_t alternate;
} GPIODesc_t;

#define GPIO_DESC_CREATE(ID, MODE, PULL, SPEED, ALTERNATE) \
    { .id = (ID), .mode = (MODE), .pull = (PULL), .speed = (SPEED), .alternate = (ALTERNATE) }
#define GPIO_DESC_CREATE_SPI_DPIN(GPIO_ID, GPIO_AF) \
    GPIO_DESC_CREATE ((GPIO_ID), GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_VERY_HIGH, (GPIO_AF))
#define GPIO_DESC_CREATE_SPI_NSS(GPIO_ID) \
    GPIO_DESC_CREATE ((GPIO_ID), GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_VERY_HIGH, 0)
#define GPIO_DESC_CREATE_UART_DPIN(GPIO_ID, GPIO_AF) \
    GPIO_DESC_CREATE ((GPIO_ID), GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_VERY_HIGH, (GPIO_AF))
#define GPIO_DESC_CREATE_I2C_DPIN(GPIO_ID, GPIO_AF) \
    GPIO_DESC_CREATE ((GPIO_ID), GPIO_MODE_AF_OD, GPIO_PULLUP, GPIO_SPEED_FREQ_VERY_HIGH, (GPIO_AF))
#define GPIO_DESC_CREATE_TIM_PIN(GPIO_ID, GPIO_AF) \
    GPIO_DESC_CREATE ((GPIO_ID), GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, (GPIO_AF))
#define GPIO_DESC_GET_ID(pGPIO_DESC)        ((pGPIO_DESC)->id)
#define GPIO_DESC_GET_MODE(pGPIO_DESC)      ((pGPIO_DESC)->mode)
#define GPIO_DESC_GET_PULL(pGPIO_DESC)      ((pGPIO_DESC)->pull)
#define GPIO_DESC_GET_SPEED(pGPIO_DESC)     ((pGPIO_DESC)->speed)
#define GPIO_DESC_GET_ALTERNATE(pGPIO_DESC) ((pGPIO_DESC)->alternate)

typedef struct {
    eTIMER_ID_t timerId;
    GPIODesc_t gpioDesc;
} TimerDesc_t;

#define TIM_DESC_CREATE(TIMER_ID, GPIO_ID, GPIO_AF) \
    { .timerId = (TIMER_ID), .gpioDesc = GPIO_DESC_CREATE_TIM_PIN ((GPIO_ID), (GPIO_AF)) }
#define TIM_DESC_GET_ID(pTIMER_DESC)   ((pTIMER_DESC)->timerId)
#define TIM_DESC_GET_GPIO(pTIMER_DESC) (&((pTIMER_DESC)->gpioDesc))

typedef struct {
    eDEVICE_ID_t deviceId;
    GPIODesc_t nssDesc;
} SPIDeviceDesc_t;

#define SPI_DEV_DESC_GET_ID(pSPIDEV_DESC)  ((pSPIDEV_DESC)->deviceId)
#define SPI_DEV_DESC_GET_NSS(pSPIDEV_DESC) (&((pSPIDEV_DESC)->nssDesc))

typedef struct {
    eSPI_SPEED_t speed;
    GPIODesc_t sckDesc;
    GPIODesc_t misoDesc;
    GPIODesc_t mosiDesc;
    SPIDeviceDesc_t connectedDevices[SPI_MAX_DEVICES_PER_BUS];
    uint8_t nConnectedDevices;
} SPIDesc_t;

#define SPI_DESC_GET_SCK(pSPI_DESC)                (&((pSPI_DESC)->sckDesc))
#define SPI_DESC_GET_MISO(pSPI_DESC)               (&((pSPI_DESC)->misoDesc))
#define SPI_DESC_GET_MOSI(pSPI_DESC)               (&((pSPI_DESC)->mosiDesc))
#define SPI_DESC_GET_CONNECTED_DEVS(pSPI_DESC)     ((pSPI_DESC)->connectedDevices)
#define SPI_DESC_GET_NUM_CONNECTED_DEVS(pSPI_DESC) ((pSPI_DESC)->nConnectedDevices)
#define SPI_DESC_GET_SPEED(pSPI_DESC)              ((pSPI_DESC)->speed)

typedef struct {
    GPIODesc_t txDesc;
    GPIODesc_t rxDesc;
    eUART_BAUDRATE_t baudRate;
} UARTDesc_t;

#define UART_DESC_GET_TX(pUART_DESC)       (&((pUART_DESC)->txDesc))
#define UART_DESC_GET_RX(pUART_DESC)       (&((pUART_DESC)->rxDesc))
#define UART_DESC_GET_BAUDRATE(pUART_DESC) ((pUART_DESC)->baudRate)

typedef struct {
    GPIODesc_t sclDesc;
    GPIODesc_t sdaDesc;
} I2CDesc_t;

#define I2C_DESC_GET_SCL(pI2C_DESC) (&((pI2C_DESC)->sclDesc))
#define I2C_DESC_GET_SDA(pI2C_DESC) (&((pI2C_DESC)->sdaDesc))

typedef struct {
    eBUS_ID_t busId;
    union {
        I2CDesc_t i2cDesc;
        SPIDesc_t spiDesc;
        UARTDesc_t uartDesc;
    };
} BusDesc_t;

#define BUS_DESC_GET_ID(pBUS_DESC)      ((pBUS_DESC)->busId)
#define BUS_DESC_IS_I2C(pBUS_DESC)      (BUS_ID_IS_I2C (BUS_DESC_GET_ID (pBUS_DESC)))
#define BUS_DESC_IS_SPI(pBUS_DESC)      (BUS_ID_IS_SPI (BUS_DESC_GET_ID (pBUS_DESC)))
#define BUS_DESC_IS_UART(pBUS_DESC)     (BUS_ID_IS_UART (BUS_DESC_GET_ID (pBUS_DESC)))

#define BUS_DESC_GET_I2C(pBUS_DESC)     (&((pBUS_DESC)->i2cDesc))
#define BUS_DESC_I2C_GET_SCL(pBUS_DESC) (I2C_DESC_GET_SCL (BUS_DESC_GET_I2C (pBUS_DESC)))
#define BUS_DESC_I2C_GET_SDA(pBUS_DESC) (I2C_DESC_GET_SDA (BUS_DESC_GET_I2C (pBUS_DESC)))

// clang-format off
#define BUS_DESC_GET_SPI(pBUS_DESC)                     (&((pBUS_DESC)->spiDesc))
#define BUS_DESC_SPI_GET_SCK(pBUS_DESC)                 (SPI_DESC_GET_SCK (BUS_DESC_GET_SPI (pBUS_DESC)))
#define BUS_DESC_SPI_GET_MISO(pBUS_DESC)                (SPI_DESC_GET_MISO (BUS_DESC_GET_SPI (pBUS_DESC)))
#define BUS_DESC_SPI_GET_MOSI(pBUS_DESC)                (SPI_DESC_GET_MOSI (BUS_DESC_GET_SPI (pBUS_DESC)))
#define BUS_DESC_SPI_GET_CONNECTED_DEVS(pBUS_DESC)      (SPI_DESC_GET_CONNECTED_DEVS (BUS_DESC_GET_SPI (pBUS_DESC)))
#define BUS_DESC_SPI_GET_NUM_CONNECTED_DEVS(pBUS_DESC)  (SPI_DESC_GET_NUM_CONNECTED_DEVS (BUS_DESC_GET_SPI (pBUS_DESC)))
#define BUS_DESC_SPI_GET_SPEED(pBUS_DESC)               (SPI_DESC_GET_SPEED (BUS_DESC_GET_SPI (pBUS_DESC)))

#define BUS_DESC_GET_UART(pBUS_DESC)    (&((pBUS_DESC)->uartDesc))
#define BUS_DESC_UART_GET_TX(pBUS_DESC) (UART_DESC_GET_TX (BUS_DESC_GET_UART (pBUS_DESC)))
#define BUS_DESC_UART_GET_RX(pBUS_DESC) (UART_DESC_GET_RX (BUS_DESC_GET_UART (pBUS_DESC)))
#define BUS_DESC_UART_GET_BAUDRATE(pBUS_DESC) (UART_DESC_GET_BAUDRATE (BUS_DESC_GET_UART (pBUS_DESC)))
// clang-format on

typedef struct {
    eEXTI_ID_t extiId;
    GPIODesc_t gpioDesc;
} EXTIDesc_t;

#define EXTI_DESC_GET_ID(pEXTI_DESC)   ((pEXTI_DESC)->extiId)
#define EXTI_DESC_GET_GPIO(pEXTI_DESC) (&((pEXTI_DESC)->gpioDesc))

typedef struct {
    BusDesc_t* pBusDesc;
    EXTIDesc_t extiDesc;
} GenDevDesc_t;

// clang-format off

#define GENDEV_DESC_GET_EXTI(pGENDEV_DESC) (&((pGENDEV_DESC)->extiDesc))
#define GENDEV_DESC_GET_EXTI_ID(pGENDEV_DESC) (EXTI_DESC_GET_ID (&GENDEV_DESC_GET_EXTI (pGENDEV_DESC)))
#define GENDEV_DESC_GET_EXTI_GPIO(pGENDEV_DESC) (EXTI_DESC_GET_GPIO (&GENDEV_DESC_GET_EXTI (pGENDEV_DESC)))

#define GENDEV_DESC_GET_BUS(pGENDEV_DESC)  ((pGENDEV_DESC)->pBusDesc)
#define GENDEV_DESC_GET_BUS_ID(pGENDEV_DESC) (BUS_DESC_GET_ID (GENDEV_DESC_GET_BUS (pGENDEV_DESC)))
#define GENDEV_DESC_GET_BUS_I2C(pGENDEV_DESC) (BUS_DESC_GET_I2C (GENDEV_DESC_GET_BUS (pGENDEV_DESC)))
#define GENDEV_DESC_GET_BUS_SPI(pGENDEV_DESC) (BUS_DESC_GET_SPI (GENDEV_DESC_GET_BUS (pGENDEV_DESC)))
#define GENDEV_DESC_GET_BUS_UART(pGENDEV_DESC) (BUS_DESC_GET_UART (GENDEV_DESC_GET_BUS (pGENDEV_DESC)))

// clang-format on

typedef struct {
    ePID_TYPE_t pidType;
    union {
        struct {
            float rollMix;
            float pitchMix;
            float yawMix;
        } angle;

        struct {
            float empty;
        } rate;
    };
} PIDDesc_t;

#define PID_DESC_ANGLE_CREATE(ROLL_MIX, PITCH_MIX, YAW_MIX) \
    {                                                       \
        .pidType = ePID_TYPE_ANGLE, .angle = {              \
            .rollMix  = (ROLL_MIX),                         \
            .pitchMix = (PITCH_MIX),                        \
            .yawMix   = (YAW_MIX)                           \
        }                                                   \
    }
#define PID_DESC_GET_TYPE(pPID_DESC)      ((pPID_DESC)->pidType)
#define PID_DESC_GET_ROLL_MIX(pPID_DESC)  ((pPID_DESC)->angle.rollMix)
#define PID_DESC_GET_PITCH_MIX(pPID_DESC) ((pPID_DESC)->angle.pitchMix)
#define PID_DESC_GET_YAW_MIX(pPID_DESC)   ((pPID_DESC)->angle.yawMix)

typedef struct {
    eACTUATOR_PROTOCOL_t protType;
    union {
        struct {
            uint32_t pwmFreq;
        } pwm;

        struct {
            eDSHOT_TYPE_t type;
            eDSHOT_SPEED_t speed;
        } dshot;
    };
} ActProtDesc_t;

#define ACTPROT_DESC_PWM_CREATE(PWM_FREQUENCY)                                   \
    {                                                                            \
        .protType = eACTUATOR_PROTOCOL_PWM, .pwm = {.pwmFreq = (PWM_FREQUENCY) } \
    }
#define ACTPROT_DESC_DSHOT_CREATE(DSHOT_TYPE, DSHOT_SPEED) \
    {                                                      \
        .protType = eACTUATOR_PROTOCOL_DSHOT, .dshot = {   \
            .type  = (DSHOT_TYPE),                         \
            .speed = (DSHOT_SPEED)                         \
        }                                                  \
    }
#define ACTPROT_DESC_GET_TYPE(pACTUATOR_DESC)        ((pACTUATOR_DESC)->protType)
#define ACTPROT_DESC_GET_PWM_FREQ(pACTUATOR_DESC)    ((pACTUATOR_DESC)->pwm.pwmFreq)
#define ACTPROT_DESC_GET_DSHOT_TYPE(pACTUATOR_DESC)  ((pACTUATOR_DESC)->dshot.type)
#define ACTPROT_DESC_GET_DSHOT_SPEED(pACTUATOR_DESC) ((pACTUATOR_DESC)->dshot.speed)

typedef struct DevDesc_s DevDesc_t;
typedef struct ActDevDesc_s {
    DevDesc_t* pLinkedDesc;
    union {
        TimerDesc_t timerDesc;
        GPIODesc_t gpioDesc;
    };
    PIDDesc_t pidDesc;
    ActProtDesc_t protDesc;
} ActDevDesc_t;

#define ACTDEV_DESC_CREATE_PWM_ANGLE(pTIMER, pLINKED_ACTUATOR, PID_ROLL, PID_PITCH, PID_YAW, PWM_FREQUENCY) \
    { .pTimerDesc   = (pTIMER),                                                                             \
      .pLinkedDesc  = (pLINKED_ACTUATOR),                                                                   \
      .pidType      = ePID_TYPE_ANGLE,                                                                      \
      .pidDesc      = PID_DESC_ANGLE_CREATE ((PID_ROLL), (PID_PITCH), (PID_YAW)),                           \
      .protType     = eACTUATOR_PROTOCOL_PWM,                                                               \
      .protocolDesc = ACTPROT_DESC_PWM_CREATE (PWM_FREQUENCY) }

#define ACTDEV_DESC_CREATE_DSHOT_ANGLE(pTIMER, pLINKED_ACTUATOR, PID_ROLL, PID_PITCH, PID_YAW, DSHOT_TYPE, DSHOT_SPEED) \
    { .pTimerDesc   = (pTIMER),                                                                                         \
      .pLinkedDesc  = (pLINKED_ACTUATOR),                                                                               \
      .pidType      = ePID_TYPE_ANGLE,                                                                                  \
      .pidDesc      = PID_DESC_ANGLE_CREATE ((PID_ROLL), (PID_PITCH), (PID_YAW)),                                       \
      .protType     = eACTUATOR_PROTOCOL_DSHOT,                                                                         \
      .protocolDesc = ACTPROT_DESC_DSHOT_CREATE (DSHOT_TYPE, DSHOT_SPEED) }
// clang-format off
#define ACTDEV_DESC_GET_TIM(pACTUATOR_DESC)         (&((pACTUATOR_DESC)->timerDesc))
#define ACTDEV_DESC_GET_TIM_ID(pACTUATOR_DESC)      (TIM_DESC_GET_ID (ACTDEV_DESC_GET_TIM (pACTUATOR_DESC)))
#define ACTDEV_DESC_GET_TIM_GPIO(pACTUATOR_DESC)    (TIM_DESC_GET_GPIO(ACTDEV_DESC_GET_TIM (pACTUATOR_DESC)))
#define ACTDEV_DESC_GET_LINKED(pACTUATOR_DESC)        ((pACTUATOR_DESC)->pLinkedDesc)
#define ACTDEV_DESC_GET_PID_TYPE(pACTUATOR_DESC)      ((pACTUATOR_DESC)->pidType)
#define ACTDEV_DESC_GET_PID_DESC(pACTUATOR_DESC)      (&((pACTUATOR_DESC)->pidDesc))
#define ACTDEV_DESC_GET_PROT_DESC(pACTUATOR_DESC) (&((pACTUATOR_DESC)->protDesc))
#define ACTDEV_DESC_GET_PROT_TYPE(pACTUATOR_DESC) (ACTDEV_DESC_GET_PROT_DESC(pACTUATOR_DESC)->protType)

// clang-format on
#define ACTDEV_DESC_GET_PWM_FREQ(pACTUATOR_DESC) \
    (ACTPROT_DESC_GET_PWM_FREQ (&ACTDEV_DESC_GET_PROT_DESC (pACTUATOR_DESC)))
#define ACTDEV_DESC_GET_DSHOT_TYPE(pACTUATOR_DESC) \
    (ACTPROT_DESC_GET_DSHOT_TYPE (&ACTDEV_DESC_GET_PROT_DESC (pACTUATOR_DESC)))
#define ACTDEV_DESC_GET_DSHOT_SPEED(pACTUATOR_DESC) \
    (ACTPROT_DESC_GET_DSHOT_SPEED (&ACTDEV_DESC_GET_PROT_DESC (pACTUATOR_DESC)))
typedef struct DevDesc_s {
    eDEVICE_ID_t deviceId;
    eDEVICE_FLAGS_t flags;
    bool isRequired;
    union {
        ActDevDesc_t actDev;
        GenDevDesc_t genDev;
    };
} DevDesc_t;

#define DEV_DESC_GET_ID(pDEV_DESC)      ((pDEV_DESC)->deviceId)
#define DEV_DESC_GET_FLAGS(pDEV_DESC)   ((pDEV_DESC)->flags)
#define DEV_DESC_IS_REQUIRED(pDEV_DESC) ((pDEV_DESC)->isRequired)

#define DEV_DESC_IS_GENDEV(pDEV_DESC) \
    (!DEVICE_ID_IS_MOTOR (DEV_DESC_GET_ID (pDEV_DESC)) && !DEVICE_ID_IS_SERVO (DEV_DESC_GET_ID (pDEV_DESC)))
#define DEV_DESC_GET_GENDEV(pDEV_DESC)      (&((pDEV_DESC)->genDev))
#define DEV_DESC_GET_GENDEV_BUS(pDEV_DESC)  (GENDEV_DESC_GET_BUS (DEV_DESC_GET_GENDEV (pDEV_DESC)))
#define DEV_DESC_GET_GENDEV_EXTI(pDEV_DESC) (GENDEV_DESC_GET_EXTI (DEV_DESC_GET_GENDEV (pDEV_DESC)))

#define DEV_DESC_IS_ACTDEV(pDEV_DESC) \
    (DEVICE_ID_IS_MOTOR (DEV_DESC_GET_ID (pDEV_DESC)) || DEVICE_ID_IS_SERVO (DEV_DESC_GET_ID (pDEV_DESC)))
#define DEV_DESC_GET_ACTDEV(pDEV_DESC) (&((pDEV_DESC)->actDev))
// clang-format off
#define DEV_DESC_GET_ACTDEV_TIM(pDEV_DESC)          (ACTDEV_DESC_GET_TIM (DEV_DESC_GET_ACTDEV (pDEV_DESC)))
#define DEV_DESC_GET_ACTDEV_LINKED(pDEV_DESC)       (ACTDEV_DESC_GET_LINKED (DEV_DESC_GET_ACTDEV (pDEV_DESC)))
#define DEV_DESC_GET_ACTDEV_PID_TYPE(pDEV_DESC)     (ACTDEV_DESC_GET_PID_TYPE (DEV_DESC_GET_ACTDEV (pDEV_DESC)))
#define DEV_DESC_GET_ACTDEV_PID_DESC(pDEV_DESC)     (ACTDEV_DESC_GET_PID_DESC (DEV_DESC_GET_ACTDEV (pDEV_DESC)))
#define DEV_DESC_GET_ACTDEV_PROT_TYPE(pDEV_DESC)    (ACTDEV_DESC_GET_PROT_TYPE (DEV_DESC_GET_ACTDEV (pDEV_DESC)))
#define DEV_DESC_GET_ACTDEV_PROT_DESC(pDEV_DESC)    (ACTDEV_DESC_GET_PROT_DESC (DEV_DESC_GET_ACTDEV (pDEV_DESC)))
#define DEV_DESC_GET_ACTDEV_PWM_FREQ(pDEV_DESC)     (ACTDEV_DESC_GET_PWM_FREQ (DEV_DESC_GET_ACTDEV (pDEV_DESC)))

#define DEV_DESC_GET_PID_DESC(pDEV_DESC)     (DEV_DESC_GET_ACTDEV_PID_DESC (pDEV_DESC))

#define DEV_DESC_HAS_TIM(pDEV_DESC) (DEV_DESC_IS_ACTDEV(pDEV_DESC) && DEV_DESC_GET_ACTDEV_TIM(pDEV_DESC) != NULL)
#define DEV_DESC_GET_TIM(pDEV_DESC) (DEV_DESC_GET_ACTDEV_TIM(pDEV_DESC))
#define DEV_DESC_GET_TIM_ID(pDEV_DESC) (ACTDEV_DESC_GET_TIM_ID(DEV_DESC_GET_ACTDEV(pDEV_DESC)))
#define DEV_DESC_GET_TIM_GPIO(pDEV_DESC) (ACTDEV_DESC_GET_TIM_GPIO(DEV_DESC_GET_ACTDEV(pDEV_DESC)))
#define DEV_DESC_GET_TIM_GPIO_ID(pDEV_DESC) (GPIO_DESC_GET_ID(DEV_DESC_GET_TIM_GPIO(pDEV_DESC)))    
#define DEV_DESC_GET_TIM_GPIO_MODE(pDEV_DESC) (GPIO_DESC_GET_MODE(DEV_DESC_GET_TIM_GPIO(pDEV_DESC)))
#define DEV_DESC_GET_TIM_GPIO_PULL(pDEV_DESC) (GPIO_DESC_GET_PULL(DEV_DESC_GET_TIM_GPIO(pDEV_DESC)))
#define DEV_DESC_GET_TIM_GPIO_SPEED(pDEV_DESC) (GPIO_DESC_GET_SPEED(DEV_DESC_GET_TIM_GPIO(pDEV_DESC)))
#define DEV_DESC_GET_TIM_GPIO_ALTERNATE(pDEV_DESC) (GPIO_DESC_GET_ALTERNATE(DEV_DESC_GET_TIM_GPIO(pDEV_DESC)))

#define DEV_DESC_GET_PROT_DESC(pDEV_DESC)    (DEV_DESC_GET_ACTDEV_PROT_DESC (pDEV_DESC))
#define DEV_DESC_GET_PROT_TYPE(pDEV_DESC)    (DEV_DESC_GET_ACTDEV_PROT_TYPE (pDEV_DESC))
#define DEV_DESC_IS_PROT_PWM(pDEV_DESC)  (DEV_DESC_GET_PROT_TYPE(pDEV_DESC) == eACTUATOR_PROTOCOL_PWM)
#define DEV_DESC_GET_PROT_PWM_FREQ(pDEV_DESC) (ACTDEV_DESC_GET_PWM_FREQ(DEV_DESC_GET_ACTDEV(pDEV_DESC)))
#define DEV_DESC_IS_PROT_DSHOT(pDEV_DESC) (DEV_DESC_GET_PROT_TYPE(pDEV_DESC) == eACTUATOR_PROTOCOL_DSHOT)
#define DEV_DESC_GET_PROT_DSHOT_TYPE(pDEV_DESC) (ACTPROT_DESC_GET_DSHOT_TYPE (DEV_DESC_GET_PROT_DESC (pDEV_DESC)))
#define DEV_DESC_GET_PROT_DSHOT_SPEED(pDEV_DESC) (ACTPROT_DESC_GET_DSHOT_SPEED (DEV_DESC_GET_PROT_DESC (pDEV_DESC)))

#define DEV_DESC_GET_BUS(pDEV_DESC)  (GENDEV_DESC_GET_BUS (DEV_DESC_GET_GENDEV (pDEV_DESC)))
#define DEV_DESC_GET_EXTI(pDEV_DESC) (GENDEV_DESC_GET_EXTI (DEV_DESC_GET_GENDEV (pDEV_DESC)))
#define DEV_DESC_GET_BUS_ID(pDEV_DESC) (BUS_DESC_GET_ID (DEV_DESC_GET_GENDEV_BUS (pDEV_DESC)))
#define DEV_DESC_HAS_BUS(pDEV_DESC) (DEV_DESC_IS_GENDEV(pDEV_DESC) && DEV_DESC_GET_BUS (pDEV_DESC) != NULL)

// clang-format on

typedef struct {
    DevDesc_t** ppDeviceDescs;
    uint8_t numDevices;
    bool isInitialized;
} DeviceTree_t;

eSTATUS_t DeviceTree_Init (void);
eSTATUS_t DeviceTree_InitImpl (DeviceTree_t* pOutDeviceTree);
// eSTATUS_t BoardConfInit_MyBoard (DeviceTree_t* pOutDeviceTree);
DeviceTree_t* DeviceTree_Get (void);
DevDesc_t* DeviceTree_GetDeviceById (eDEVICE_ID_t deviceId);

#endif // CONF_BASE_H