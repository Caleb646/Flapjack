#ifndef PID_H
#define PID_H

#include <stdint.h>
#include "stm32h7xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"

#include "imu.h"

typedef struct
{
	IMU *pIMU;
} PIDVelocityUpdate;

void PIDTaskCalcNewVelFromIMU(void *pvParameters);
int8_t PIDVelInit();

#endif // PID_H
