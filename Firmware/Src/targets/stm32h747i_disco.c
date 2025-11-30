#include <stdint.h>

#include "targets/stm32h747i_disco.h"

#include "cfg/imu/imu.h"


void Target_Init (void) {

    IMU_CFG_INIT (0, eIMU_ALIGNMENT_DEFAULT, 1000U);
    IMU_CFG_SET_SPI (0, eSPI_2_BUS_ID, GPIO_ID_MAKE (eGPIO_PORTID_A, eGPIO_PINID_11));
}