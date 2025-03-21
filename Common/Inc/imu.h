#ifndef IMU_H
#define IMU_H

#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_spi.h"
#include "stdint.h"

#define BIT_ISSET(v, bit) ((v & bit) > 0)

typedef struct
{
	SPI_HandleTypeDef* pSPI;
	int16_t ax, ay, az, gx, gy, gz;
} IMU;

int8_t IMUReadReg(IMU *pIMU, uint8_t const reg, uint8_t *pBuf, uint32_t len);
int8_t IMUWriteReg(IMU *pIMU, uint8_t const reg, uint8_t *pBuf, uint32_t len);
int8_t IMUInit(IMU *pIMU, SPI_HandleTypeDef* pSPI);

#endif // IMU_H
