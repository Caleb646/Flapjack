#ifndef TARGET_H
#define TARGET_H

// #pragma GCC diagnostic push
// #pragma GCC diagnostic ignored "-Wmacro-redefined"

// to disable redefinition warnings for default target macros
#pragma GCC system_header

// clang-format off
#define TARG_ZERO 0U
#define TARG_EMPTY
#define TARG_PIN_NONE NONE
#define TARG_PIN_EXISTS_EXPAND(PORT, ...)               (GPIO_ID_MAKE_PORTID (PORT) != TARG_ZERO)
#define TARG_PIN_EXISTS(...)                            TARG_PIN_EXISTS_EXPAND(__VA_ARGS__)
#define TARG_SERIAL_PORT_EXISTS_EXPAND(SERIAL)          (SERIAL_PORT_ID_MAKE(SERIAL) != TARG_ZERO)
#define TARG_SERIAL_PORT_EXISTS(...)                    TARG_SERIAL_PORT_EXISTS_EXPAND(__VA_ARGS__)

#define TARG_USE_DUAL_CORE                                      TARG_ZERO
#define TARG_PRIMARY_CORE                                        TARG_ZERO
#define TARG_SECONDARY_CORE                                      TARG_ZERO
#define TARG_DUAL_CORE_ENABLED()                                 (TARG_USE_DUAL_CORE != TARG_ZERO)

// TODO move to platform
#define TARG_SHARED_MEM_BSS_SECTION                              TARG_EMPTY
#define TARG_SHARED_MEM_DATA_SECTION                             TARG_EMPTY

#define TARG_MAX_MOTORS                                          2
#define TARG_MAX_SERVOS                                          8
#define TARG_MAX_ACCS                                            1
#define TARG_MAX_GYROS                                           1
#define TARG_MAX_MAGS                                            1
#define TARG_MAX_GPS                                             1
#define TARG_MAX_TIMS                                            16

#define TARG_SPI_1_SCK                                           TARG_PIN_NONE
#define TARG_SPI_1_MISO                                          TARG_PIN_NONE
#define TARG_SPI_1_MOSI                                          TARG_PIN_NONE

#define TARG_SPI_2_SCK                                           TARG_PIN_NONE
#define TARG_SPI_2_MISO                                          TARG_PIN_NONE
#define TARG_SPI_2_MOSI                                          TARG_PIN_NONE

#define TARG_SPI_3_SCK                                           TARG_PIN_NONE // port, pin -- Example: A, 1
#define TARG_SPI_3_MISO                                          TARG_PIN_NONE // port, pin -- Example: A, 1
#define TARG_SPI_3_MOSI                                          TARG_PIN_NONE // port, pin -- Example: A, 1

#define TARG_SPI_4_SCK                                           TARG_PIN_NONE // port, pin -- Example: A, 1
#define TARG_SPI_4_MISO                                          TARG_PIN_NONE // port, pin -- Example: A, 1
#define TARG_SPI_4_MOSI                                          TARG_PIN_NONE // port, pin -- Example: A, 1

#define TARG_SPI_5_SCK                                           TARG_PIN_NONE // port, pin -- Example: A, 1
#define TARG_SPI_5_MISO                                          TARG_PIN_NONE // port, pin -- Example: A, 1
#define TARG_SPI_5_MOSI                                          TARG_PIN_NONE // port, pin -- Example: A, 1

// clang-format off
#define TARG_SPI_ENABLED(SPI_ID) (TARG_PIN_EXISTS(TARG_SPI_##SPI_ID##_SCK) && TARG_PIN_EXISTS(TARG_SPI_##SPI_ID##_MISO) && TARG_PIN_EXISTS(TARG_SPI_##SPI_ID##_MOSI))
// clang-format on

#define TARG_UART_1_TX                                            TARG_PIN_NONE // port, pin -- Example: A, 1
#define TARG_UART_1_RX                                            TARG_PIN_NONE // port, pin -- Example: A, 1

#define TARG_UART_2_TX                                            TARG_PIN_NONE // port, pin -- Example: A, 1
#define TARG_UART_2_RX                                            TARG_PIN_NONE // port, pin -- Example: A, 1

#define TARG_UART_3_TX                                            TARG_PIN_NONE // port, pin -- Example: A, 1
#define TARG_UART_3_RX                                            TARG_PIN_NONE // port, pin -- Example: A, 1

#define TARG_UART_4_TX                                            TARG_PIN_NONE
#define TARG_UART_4_RX                                            TARG_PIN_NONE

#define TARG_UART_5_TX                                            TARG_PIN_NONE // port, pin -- Example: A, 1
#define TARG_UART_5_RX                                            TARG_PIN_NONE // port, pin -- Example: A, 1

// clang-format off
#define TARG_UART_ENABLED(UART_ID)          (TARG_PIN_EXISTS(TARG_UART_##UART_ID##_TX) && TARG_PIN_EXISTS(TARG_UART_##UART_ID##_RX))
// clang-format on

// #define TARG_TIM_1_CH1                                            TARG_PIN_NONE // port, pin -- Example: A, 1
// #define TARG_TIM_1_CH2                                            TARG_PIN_NONE // port, pin -- Example: A, 1
// #define TARG_TIM_1_CH3                                            TARG_PIN_NONE // port, pin -- Example: A, 1
// #define TARG_TIM_1_CH4                                            TARG_PIN_NONE // port, pin -- Example: A, 1

// #define TARG_TIM_8_CH1                                            TARG_PIN_NONE // port, pin -- Example: A, 1
// #define TARG_TIM_8_CH2                                            TARG_PIN_NONE // port, pin -- Example: A, 1
// #define TARG_TIM_8_CH3                                            TARG_PIN_NONE // port, pin -- Example: A, 1
// #define TARG_TIM_8_CH4                                            TARG_PIN_NONE // port, pin -- Example: A, 1

// #define TARG_TIM_12_CH1                                           TARG_PIN_NONE // port, pin -- Example: A, 1
// #define TARG_TIM_12_CH2                                           TARG_PIN_NONE // port, pin -- Example: A, 1
// #define TARG_TIM_12_CH3                                           TARG_PIN_NONE // port, pin -- Example: A, 1
// #define TARG_TIM_12_CH4                                           TARG_PIN_NONE // port, pin -- Example: A, 1

// #define TARG_TIM_13_CH1                                           TARG_PIN_NONE // port, pin -- Example: A, 1
// #define TARG_TIM_13_CH2                                           TARG_PIN_NONE // port, pin -- Example: A, 1
// #define TARG_TIM_13_CH3                                           TARG_PIN_NONE // port, pin -- Example: A, 1
// #define TARG_TIM_13_CH4                                           TARG_PIN_NONE // port, pin -- Example: A, 1

#define TARG_SERIAL_DEBUG                                         TARG_NONE // serial port id (UART_1, UART_2....)
#define TARG_SERIAL_DEBUG_BAUD                                    115200 // optional
#define TARG_SERIAL_DEBUG_TX_BUFFER_SIZE                          2048   // optional
#define TARG_SERIAL_DEBUG_ENABLED()                               TARG_SERIAL_PORT_EXISTS (TARG_SERIAL_DEBUG)

// TODO
#define TARG_FLASH_ENABLED()                                      (0 != 0)

#define TARG_ACC                                                  TARG_ZERO // interface id -- Example: BMI323
#define TARG_ACC_SPI                                              TARG_ZERO // spi id -- Example: SPI_1
#define TARG_ACC_SPI_NSS                                          TARG_PIN_NONE // nss port, pin -- Example: A, 1
#define TARG_ACC_I2C                                              TARG_ZERO // i2c instance -- Example: I2C5
#define TARG_ACC_I2C_ADDR                                         TARG_ZERO // i2c address -- Example: 0x40
/*
 * The degrees of rotation required to align the acc X axis with the body frame's F (forward) axis.
 * If the X axis is -90 degrees from F, i.e X is pointing left in the FRU frame, then a positive 90
 * clockwise yaw rotation is required to correct it. So TARG_ACC_ALIGN_F 0, 0, 90
 */
// clang-format off
#define TARG_ACC_ALIGN_F                                         0, 0, 0 // degrees (-180 to 180) to rotate X via roll (F), pitch (R), yaw (U)
#define TARG_ACC_ALIGN_R                                         0, 0, 0 // degrees (-180 to 180) to rotate Y
#define TARG_ACC_ALIGN_U                                         0, 0, 0 // degrees (-180 to 180) to rotate Z
#define TARG_ACC_SPI_ENABLED()                                  TARG_PIN_EXISTS(TARG_ACC_SPI_NSS)
#define TARG_ACC_I2C_ENABLED()                                  (TARG_ACC_I2C_ADDR != TARG_ZERO)
#define TARG_ACC_ENABLED()                                      (TARG_ACC_SPI_ENABLED() || TARG_ACC_I2C_ENABLED())

#define TARG_GYRO                                                TARG_ZERO // interface id -- Example: BMI323
#define TARG_GYRO_SPI                                            TARG_ZERO
#define TARG_GYRO_SPI_NSS                                         TARG_PIN_NONE
#define TARG_GYRO_I2C                                            TARG_ZERO 
#define TARG_GYRO_I2C_ADDR                                        TARG_ZERO // i2c address -- Example: 0x40
#define TARG_GYRO_ALIGN_F                                        0, 0, 0 // degrees (-180 to 180) to rotate X
#define TARG_GYRO_ALIGN_R                                        0, 0, 0 // degrees (-180 to 180) to rotate Y
#define TARG_GYRO_ALIGN_U                                        0, 0, 0 // degrees (-180 to 180) to rotate Z
#define TARG_GYRO_SPI_ENABLED()                                  TARG_PIN_EXISTS(TARG_GYRO_SPI_NSS)
#define TARG_GYRO_I2C_ENABLED()                                  (TARG_GYRO_I2C_ADDR != TARG_ZERO)
#define TARG_GYRO_ENABLED()                                      (TARG_GYRO_SPI_ENABLED() || TARG_GYRO_I2C_ENABLED())

#define TARG_MAG                                                 TARG_ZERO // interface id -- Example: MMC5983
#define TARG_MAG_SPI                                             TARG_ZERO // spi id -- Example: SPI_1
#define TARG_MAG_SPI_NSS                                         TARG_PIN_NONE
#define TARG_MAG_I2C                                             TARG_ZERO 
#define TARG_MAG_I2C_ADDR                                        TARG_ZERO // i2c address -- Example: 0x40
#define TARG_MAG_ALIGN_F                                         0, 0, 0 // degrees (-180 to 180) to rotate X
#define TARG_MAG_ALIGN_R                                         0, 0, 0 // degrees (-180 to 180) to rotate Y
#define TARG_MAG_ALIGN_U                                         0, 0, 0 // degrees (-180 to 180) to rotate Z
#define TARG_MAG_SPI_ENABLED()                                  TARG_PIN_EXISTS(TARG_MAG_SPI_NSS)
#define TARG_MAG_I2C_ENABLED()                                  (TARG_MAG_I2C_ADDR != TARG_ZERO)
#define TARG_MAG_ENABLED()                                      (TARG_MAG_SPI_ENABLED() || TARG_MAG_I2C_ENABLED())

// TODO
#define TARG_GPS_ENABLED()                                      (0 != 0)

// TODO
#define TARG_BARO_ENABLED()                                      (0 != 0)

#define TARG_MOTOR_PROT                                          DSHOT_150 // eMOTOR_PROT_t
#define TARG_MOTOR_1_PIN                                         TARG_PIN_NONE // port, pin -- Example: B, 1
#define TARG_MOTOR_2_PIN                                         TARG_PIN_NONE // port, pin -- Example: B, 1
#define TARG_MOTOR_3_PIN                                         TARG_PIN_NONE // port, pin -- Example: B, 1
#define TARG_MOTOR_4_PIN                                         TARG_PIN_NONE // port, pin -- Example: B, 1
#define TARG_MOTOR_ENABLED(MOTOR_ID)                                TARG_PIN_EXISTS(TARG_MOTOR_##MOTOR_ID##_PIN)

#define TARG_SERVO_PROT                                          PWM  // eMOTOR_PROT_t
#define TARG_SERVO_1_PIN                                         TARG_PIN_NONE // port, pin -- Example: B, 1
#define TARG_SERVO_2_PIN                                         TARG_PIN_NONE // port, pin -- Example: B, 1
#define TARG_SERVO_3_PIN                                         TARG_PIN_NONE // port, pin -- Example: B, 1
#define TARG_SERVO_4_PIN                                         TARG_PIN_NONE // port, pin -- Example: B, 1
#define TARG_SERVO_5_PIN                                         TARG_PIN_NONE // port, pin -- Example: B, 1
#define TARG_SERVO_6_PIN                                         TARG_PIN_NONE // port, pin -- Example: B, 1
#define TARG_SERVO_7_PIN                                         TARG_PIN_NONE // port, pin -- Example: B, 1
#define TARG_SERVO_8_PIN                                         TARG_PIN_NONE // port, pin -- Example: B, 1
#define TARG_SERVO_ENABLED(SERVO_ID)                                TARG_PIN_EXISTS(TARG_SERVO_##SERVO_ID##_PIN)

// clang-format on

void Target_Init (void);

// include target board last to allow overrides
#include "targets/stm32h747i_disco.h"

// #pragma GCC diagnostic pop

#endif