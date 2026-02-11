#ifndef TARGETS_NUCLEO_H747ZI_H
#define TARGETS_NUCLEO_H747ZI_H


#define BOARD_NAME             "nucleo_h747zi"

#define SPI_2_ENABLED          1U
#define SPI_2_SCK_GPIO_PORT    GPIOA
#define SPI_2_SCK_GPIO_PIN     GPIO_PIN_12
#define SPI_2_MISO_GPIO_PORT   GPIOC
#define SPI_2_MISO_GPIO_PIN    GPIO_PIN_2
#define SPI_2_MOSI_GPIO_PORT   GPIOC
#define SPI_2_MOSI_GPIO_PIN    GPIO_PIN_3
#define SPI_2_AF               GPIO_AF5_SPI1

#define UART_1_ENABLED         1U
#define UART_1_INSTANCE        USART1
#define UART_1_TX_GPIO_PORT    GPIOA
#define UART_1_TX_GPIO_PIN     GPIO_PIN_10
#define UART_1_RX_GPIO_PORT    GPIOA
#define UART_1_RX_GPIO_PIN     GPIO_PIN_9
#define UART_1_AF              GPIO_AF7_USART1

#define TIMER_8_ENABLED        1U
#define TIMER_8_INSTANCE       TIM8
#define TIMER_8_AF             GPIO_AF3_TIM8

#define TIMER_13_ENABLED       1U
#define TIMER_13_INSTANCE      TIM13
#define TIMER_13_AF            GPIO_AF9_TIM13

#define SERIAL_DEBUG_ENABLED   1U
#define SERIAL_DEBUG_UART      UART_1
#define SERIAL_DEBUG_BAUD_RATE 230400U

#define IMU_ENABLED            1U
#define IMU_SPI_BUS_ID         eSPI_2_BUS_ID
#define IMU_SPI_NSS_GPIO_PORT  GPIOA
#define IMU_SPI_NSS_GPIO_PIN   GPIO_PIN_11

#define MOTOR_1_ENABLED        1U
#define MOTOR_1_GPIO_PORT      GPIOC
#define MOTOR_1_GPIO_PIN       GPIO_PIN_6
#define MOTOR_1_TIMER          TIMER_8
#define MOTOR_1_CHANNEL        TIM_CHANNEL_1

#define SERVO_1_ENABLED        1U
#define SERVO_1_GPIO_PORT      GPIOF
#define SERVO_1_GPIO_PIN       GPIO_PIN_8
#define SERVO_1_TIMER          TIMER_13
#define SERVO_1_CHANNEL        TIM_CHANNEL_1

#define RX_ENABLED             1U
#define RX_UART                UART_1
#define RX_UART_BAUD_RATE      230400U


#endif // TARGETS_NUCLEO_H747ZI_H