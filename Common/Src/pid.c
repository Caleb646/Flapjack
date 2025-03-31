#include "pid.h"

/*
* variables passed in:
    IMU updated accel and gyro
    Current vel and angular vel
    Desired vel and angular vel 

  variables to return:
    Change needed to vel and angular vel
*/

void PIDTaskCalcNewVelFromIMU(void *pvParameters)
{
    IMU *pIMU = (IMU*)pvParameters;
    while(1)
    {
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000));
    }
}