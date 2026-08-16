#ifndef TARGETS_BOARDS_FLAPJACK_V1_H
#define TARGETS_BOARDS_FLAPJACK_V1_H

#define BOARD_NAME             "flapjack_v1"

#define TIMER_5_ENABLED        1U
#define TIMER_5_INSTANCE       TIM5
#define TIMER_5_AF             GPIO_AF2_TIM5

#define TIMER_8_ENABLED        1U
#define TIMER_8_INSTANCE       TIM8
#define TIMER_8_AF             GPIO_AF3_TIM8

#define TIMER_12_ENABLED       1U
#define TIMER_12_INSTANCE      TIM12
#define TIMER_12_AF            GPIO_AF2_TIM12

// left motor servo
#define SERVO_1_ENABLED        1U
#define SERVO_1_GPIO_PORT      GPIOA
#define SERVO_1_GPIO_PIN       GPIO_PIN_0
#define SERVO_1_TIMER          TIMER_5
#define SERVO_1_CHANNEL        TIM_CHANNEL_1

// right motor servo
#define SERVO_2_ENABLED        1U
#define SERVO_2_GPIO_PORT      GPIOC
#define SERVO_2_GPIO_PIN       GPIO_PIN_6
#define SERVO_2_TIMER          TIMER_8
#define SERVO_2_CHANNEL        TIM_CHANNEL_1

#define MOTOR_1_ENABLED        1U
#define MOTOR_1_GPIO_PORT      GPIOH
#define MOTOR_1_GPIO_PIN       GPIO_PIN_6
#define MOTOR_1_TIMER          TIMER_12
#define MOTOR_1_CHANNEL        TIM_CHANNEL_1

#define MOTOR_2_ENABLED        1U
#define MOTOR_2_GPIO_PORT      GPIOH
#define MOTOR_2_GPIO_PIN       GPIO_PIN_9
#define MOTOR_2_TIMER          TIMER_12
#define MOTOR_2_CHANNEL        TIM_CHANNEL_2

#define SPI_1_ENABLED          1U
#define SPI_1_SCK_GPIO_PORT    GPIOA
#define SPI_1_SCK_GPIO_PIN     GPIO_PIN_5
#define SPI_1_MISO_GPIO_PORT   GPIOA
#define SPI_1_MISO_GPIO_PIN    GPIO_PIN_6
#define SPI_1_MOSI_GPIO_PORT   GPIOA
#define SPI_1_MOSI_GPIO_PIN    GPIO_PIN_7
#define SPI_1_AF               GPIO_AF5_SPI1

#define SPI_3_ENABLED               1U
#define SPI_3_SCK_GPIO_PORT         GPIOC
#define SPI_3_SCK_GPIO_PIN          GPIO_PIN_10
#define SPI_3_MISO_GPIO_PORT        GPIOC
#define SPI_3_MISO_GPIO_PIN         GPIO_PIN_11
#define SPI_3_MOSI_GPIO_PORT        GPIOC
#define SPI_3_MOSI_GPIO_PIN         GPIO_PIN_12
#define SPI_3_AF                    GPIO_AF5_SPI3

#define SPI_5_ENABLED               1U
#define SPI_5_SCK_GPIO_PORT         GPIOF
#define SPI_5_SCK_GPIO_PIN          GPIO_PIN_7
#define SPI_5_MISO_GPIO_PORT        GPIOF
#define SPI_5_MISO_GPIO_PIN         GPIO_PIN_8
#define SPI_5_MOSI_GPIO_PORT        GPIOF
#define SPI_5_MOSI_GPIO_PIN         GPIO_PIN_9
#define SPI_5_AF                    GPIO_AF5_SPI5

#define IMU_ENABLED            1U
#define IMU_SPI_BUS_ID         eSPI_1_BUS_ID
#define IMU_SPI_NSS_GPIO_PORT  GPIOC
#define IMU_SPI_NSS_GPIO_PIN   GPIO_PIN_4

#define EXT_FLASH_ENABLED           1U
#define EXT_FLASH_SPI_BUS_ID        eSPI_3_BUS_ID
#define EXT_FLASH_SPI_NSS_GPIO_PORT GPIOA
#define EXT_FLASH_SPI_NSS_GPIO_PIN  GPIO_PIN_15

#define MAG_ENABLED                 1U
#define MAG_SPI_BUS_ID              eSPI_5_BUS_ID
#define MAG_SPI_NSS_GPIO_PORT       GPIOF
#define MAG_SPI_NSS_GPIO_PIN        GPIO_PIN_4

#define BARO_ENABLED                1U
#define BARO_SPI_BUS_ID             eSPI_5_BUS_ID
#define BARO_SPI_NSS_GPIO_PORT      GPIOF
#define BARO_SPI_NSS_GPIO_PIN       GPIO_PIN_6

#define UART_1_ENABLED         1U
#define UART_1_INSTANCE        USART1
#define UART_1_TX_GPIO_PORT    GPIOB
#define UART_1_TX_GPIO_PIN     GPIO_PIN_14
#define UART_1_RX_GPIO_PORT    GPIOB
#define UART_1_RX_GPIO_PIN     GPIO_PIN_15
#define UART_1_AF                   GPIO_AF4_USART1

#define UART_2_ENABLED         1U
#define UART_2_INSTANCE        USART2
#define UART_2_TX_GPIO_PORT    GPIOD
#define UART_2_TX_GPIO_PIN     GPIO_PIN_5
#define UART_2_RX_GPIO_PORT    GPIOD
#define UART_2_RX_GPIO_PIN     GPIO_PIN_6
#define UART_2_AF              GPIO_AF7_USART2

#define UART_3_ENABLED         1U
#define UART_3_INSTANCE        USART3
#define UART_3_TX_GPIO_PORT    GPIOB
#define UART_3_TX_GPIO_PIN     GPIO_PIN_10
#define UART_3_RX_GPIO_PORT    GPIOB
#define UART_3_RX_GPIO_PIN     GPIO_PIN_11
#define UART_3_AF              GPIO_AF7_USART3

#define SERIAL_LINK_ENABLED   1U
// #define SERIAL_LINK_UART           UART_3 // UART_1
#define SERIAL_LINK_UART           UART_1
#define SERIAL_LINK_BAUD_RATE 460800U
// #define SERIAL_LINK_BAUD_RATE      9600U

#define RX_ENABLED                  1U
// #define RX_UART                  UART_2
#define RX_UART                     UART_3
#define RX_BAUD_RATE                416666U

#define GPS_ENABLED            1U
// #define GPS_UART                 UART_3
#define GPS_UART                    UART_2
#define GPS_BAUD_RATE          115200U


#endif