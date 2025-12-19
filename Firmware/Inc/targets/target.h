#ifndef TARGET_H
#define TARGET_H

#define TARG_MAX_MOTORS              2
#define TARG_MAX_SERVOS              8
#define TARG_MAX_ACCS                1
#define TARG_MAX_GYROS               1
#define TARG_MAX_MAGS                1
#define TARG_MAX_GPS                 1
#define TARG_MAX_TIMS                16

#define TARG_NONE                    0U

#define TARG_SPI_1_SCK               TARG_NONE
#define TARG_SPI_1_MISO              TARG_NONE
#define TARG_SPI_1_MOSI              TARG_NONE

#define TARG_SPI_2_SCK               TARG_NONE
#define TARG_SPI_2_MISO              TARG_NONE
#define TARG_SPI_2_MOSI              TARG_NONE

#define TARG_SPI_3_SCK               TARG_NONE
#define TARG_SPI_3_MISO              TARG_NONE
#define TARG_SPI_3_MOSI              TARG_NONE

#define TARG_SPI_4_SCK               TARG_NONE
#define TARG_SPI_4_MISO              TARG_NONE
#define TARG_SPI_4_MOSI              TARG_NONE

#define TARG_SPI_5_SCK               TARG_NONE
#define TARG_SPI_5_MISO              TARG_NONE
#define TARG_SPI_5_MOSI              TARG_NONE

#define TARG_UART_1_TX               TARG_NONE
#define TARG_UART_1_RX               TARG_NONE

#define TARG_UART_2_TX               TARG_NONE
#define TARG_UART_2_RX               TARG_NONE

#define TARG_UART_3_TX               TARG_NONE
#define TARG_UART_3_RX               TARG_NONE

#define TARG_UART_4_TX               TARG_NONE
#define TARG_UART_4_RX               TARG_NONE

#define TARG_UART_5_TX               TARG_NONE
#define TARG_UART_5_RX               TARG_NONE

#define TARG_SHARED_MEM_BSS_SECTION  __attribute__ ((section (".shared_mem_bss")))
#define TARG_SHARED_MEM_DATA_SECTION __attribute__ ((section (".shared_mem_data")))

void Target_Init (void);

// include target board last to allow overrides
#include "targets/stm32h747i_disco.h"

#endif