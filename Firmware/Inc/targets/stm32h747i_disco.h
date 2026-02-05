#ifndef STM32H747I_DISCO_H
#define STM32H747I_DISCO_H

#define BOARD_NAME                                               "STM32H747I-DISCO"

#define TARG_USE_DUAL_CORE                                       1
#define TARG_PRIMARY_CORE                                        PLAT_CORE_ID_MAKE (CM7)
#define TARG_SECONDARY_CORE                                      PLAT_CORE_ID_MAKE (CM4)

#define TARG_SHARED_MEM_BSS_SECTION                              __attribute__ ((section (".shared_mem_bss")))
#define TARG_SHARED_MEM_DATA_SECTION                             __attribute__ ((section (".shared_mem_data")))

#define TARG_SPI_1_SCK                                           A, 5
#define TARG_SPI_1_MISO                                          A, 6
#define TARG_SPI_1_MOSI                                          A, 7

#define TARG_SPI_2_SCK                                           A, 12
#define TARG_SPI_2_MISO                                          C, 2
#define TARG_SPI_2_MOSI                                          C, 3

#define TARG_UART_1_TX                                           A, 10 // port, pin -- Example: A, 1
#define TARG_UART_1_RX                                           A, 9  // port, pin -- Example: A, 1

#define TARG_TIM_12_CH1                                          C, 6
#define TARG_TIM_13_CH1                                          F, 8

#define TARG_MOTOR_1_PIN                                         TARG_TIM_12_CH1
#define TARG_SERVO_1_PIN                                         TARG_TIM_13_CH1

#define TARG_SERIAL_DEBUG                                        UART_1 // serial port id

#define TARG_ACC                                                 BMI323 // interface id -- Example: BMI323
#define TARG_ACC_SPI                                             SPI_2 // spi id -- Example: SPI_1
#define TARG_ACC_SPI_NSS                                         A, 11 // nss port, pin -- Example: A, 1
#define TARG_ACC_ALIGN_F                                         0, 0, 0
#define TARG_ACC_ALIGN_R                                         0, 0, 0
#define TARG_ACC_ALIGN_U                                         0, 0, 0


#endif // STM32H747I_DISCO_H