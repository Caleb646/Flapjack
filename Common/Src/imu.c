#include "imu.h"

int8_t IMUReadReg(IMU *pIMU, uint8_t const reg, uint8_t *pBuf, uint32_t len)
{
	uint8_t pTx[16];
	memset(pTx, 0);
	// 0x80 bmi270 read bit
	pTx[0] = 0x80 | reg;

	uint8_t pRx[16];
	memset(pRx, 0);
	// set NSS low
	// HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_RESET);
	// status = HAL_SPI_Transmit(pIMUSPIRef, pTemp, 1, 100);

	// set NSS high
	// HAL_GPIO_WritePin(SPI2_NSS_GPIO_Port, SPI2_NSS_Pin, GPIO_PIN_SET);

	if(len + 1 > 16)
	{
		return -1;
	}

	HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(pIMU->pSPI, pTx, pRx, len + 1, 100);

	if(status != HAL_OK)
	{
		return -1;
	}
	// 1st byte sent by bmi270 is a dummy byte
	memcpy(pBuf, pRx[1], len);

	return 0;
}

int8_t IMUWriteReg(IMU *pIMU, uint8_t const reg, uint8_t *pBuf, uint32_t len)
{
	uint8_t pTx[16];
	memset(pTx, 0);

	HAL_StatusTypeDef status = HAL_SPI_Transmit(pIMU->pSPI, pTx, len, 100);
}

IMU IMUInit(SPI_HandleTypeDef* pSPI)
{
	uint8_t pBuffer[10];
	// Dummy read to initialize SPI
	IMUReadReg(BMI2_CHIPID_REG, pBuffer, 1);
	IMUReadReg(BMI2_CHIPID_REG, pBuffer, 1);

	IMU imu;
	imu.pSPI = pSPI;

	if(pBuffer[0] != BMI2_CHIPID)
	{
		return imu;
	}
	return imu;
}
