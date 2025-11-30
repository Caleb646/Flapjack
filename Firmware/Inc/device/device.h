#ifndef DEVICE_DEVICE_H
#define DEVICE_DEVICE_H

#include "conf/board.h"
#include "conf/conf.h"
#include "core/core.h"
#include "core/log/logger.h"
#include "hal.h"
#include <stdint.h>
#include <string.h>


eSTATUS_t Device_InitAll (DeviceTree_t* pBoardConf);
eSTATUS_t Device_StartAll (void);

#endif // DEVICE_DEVICE_H