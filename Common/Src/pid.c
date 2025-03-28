#include "pid.h"

void PIDTaskCalcNewPositionFromIMU(void *pvParameters)
{
    IMU *pIMU = (IMU*)pvParameters;
    while(1)
    {
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000));
    }
}