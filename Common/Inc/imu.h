#ifndef IMU_H
#define IMU_H

#include <stdint.h>
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_spi.h"
#include "FreeRTOS.h"
#include "task.h"

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
	int16_t volatile rax, ray, raz, rgx, rgy, rgz;
	// millimeters per second ^ 2
	int16_t volatile ax, ay, az;
	// millidegrees per second
	int16_t volatile gx, gy, gz;
	uint8_t accRange, accODR, gyroRange, gyroODR;
	TaskHandle_t pTasktoNofityOnIMUUpdate;
	uint32_t magic;
} IMU;

void IMU2CPUInterruptHandler(IMU *pIMU);
IMU_STATUS IMUReadReg(IMU *pIMU, uint8_t const reg, uint8_t *pBuf, uint32_t len);
IMU_STATUS IMUWriteReg(IMU *pIMU, uint8_t const reg, uint8_t *pBuf, uint32_t len);
IMU_STATUS IMUInit(  
	IMU *pIMU, 
	SPI_HandleTypeDef *pSPI,
	TaskHandle_t pTasktoNofityOnIMUUpdate,
	IMU_ACC_RANGE accRange,
	IMU_ACC_ODR accODR,
	IMU_GYRO_RANGE gyroRange,
	IMU_GYRO_ODR gyroODR
);

#endif // IMU_H
