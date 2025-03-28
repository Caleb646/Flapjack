#ifndef PID_H
#define PID_H

#include <stdint.h>
#include "stm32h7xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"

#include "imu.h"


void PIDTaskCalcNewPositionFromIMU(void *pvParameters);

#endif // PID_H
