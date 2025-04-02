#ifndef IMU_H
#define IMU_H

#include <stdint.h>
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_spi.h"
#include "FreeRTOS.h"
#include "task.h"

#include "common.h"

#define IMU_MAGIC 0xFAFAAFAF

typedef enum {
	IMU_ACC_RANGE_2G  = 0x00,
	IMU_ACC_RANGE_4G  = 0x01,
	IMU_ACC_RANGE_8G  = 0x02,
	IMU_ACC_RANGE_16G = 0x03
} IMU_ACC_RANGE;

// Accelerometer output data rate in Hertz
typedef enum {
	IMU_ACC_ODR_50   = 0x07,
	IMU_ACC_ODR_100  = 0x08,
	IMU_ACC_ODR_200  = 0x09,
	IMU_ACC_ODR_400  = 0x0A,
	IMU_ACC_ODR_800  = 0x0B
} IMU_ACC_ODR;

// degrees per second
typedef enum {
	IMU_GYRO_RANGE_2000 = 0x00,
	IMU_GYRO_RANGE_1000 = 0x01,
	IMU_GYRO_RANGE_500  = 0x02,
	IMU_GYRO_RANGE_250  = 0x03,
	IMU_GYRO_RANGE_125  = 0x04
} IMU_GYRO_RANGE;

// Gyro output data rate in Hertz
typedef enum {
	IMU_GYRO_ODR_50   = 0x07,
	IMU_GYRO_ODR_100  = 0x08,
	IMU_GYRO_ODR_200  = 0x09,
	IMU_GYRO_ODR_400  = 0x0A,
	IMU_GYRO_ODR_800  = 0x0B
} IMU_GYRO_ODR;

typedef enum {
	IMU_OK,
	IMU_ERROR
} IMU_STATUS;

typedef struct
{
	SPI_HandleTypeDef* pSPI;
	Vec3 volatile rawAccel;
	Vec3 volatile rawGyro;
	int32_t volatile msLastAccUpdateTime;
	int32_t volatile msLastGyroUpdateTime;
	uint32_t accRange, accODR, gyroRange, gyroODR;
	uint32_t magic;
} IMU;

/*
* Forward declared FlightContext
*/
typedef struct FlightContext__ FlightContext;
// typedef IMU_STATUS (*imu_update_fn)(IMU *pIMU);

IMU_STATUS IMU2CPUInterruptHandler(
	IMU *pIMU, Vec3 *pOutputAccel,Vec3 *pOutputGyro
);
IMU_STATUS IMUReadReg(IMU *pIMU, uint8_t reg, uint8_t *pBuf, uint32_t len);
IMU_STATUS IMUWriteReg(IMU *pIMU, uint8_t reg, uint8_t *pBuf, uint32_t len);
IMU_STATUS IMUInit(  
	IMU *pIMU, 
	SPI_HandleTypeDef *pSPI,
	IMU_ACC_RANGE accRange,
	IMU_ACC_ODR accODR,
	IMU_GYRO_RANGE gyroRange,
	IMU_GYRO_ODR gyroODR
);

#endif // IMU_H
