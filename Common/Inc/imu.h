#ifndef IMU_H
#define IMU_H

#include "stm32h7xx_hal.h"
#include "string.h"
#include "stdint.h"

#define BMI2_CHIPID 0x24
#define BMI2_CHIPID_REG 0x00

typedef struct IMU_S
{
	SPI_HandleTypeDef* pSPI;
} IMU;

int8_t IMUReadReg(IMU *, uint8_t const reg, uint8_t *pBuf, uint32_t len);
int8_t IMUWriteReg(IMU *, uint8_t const reg, uint8_t *pBuf, uint32_t len);
IMU IMUInit(SPI_HandleTypeDef* pSPI);

#endif // IMU_H
