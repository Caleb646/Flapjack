#ifndef DEVICE_DEVICE_H
#define DEVICE_DEVICE_H

#include "common.h"
#include "conf/board.h"
#include "conf/conf.h"
#include "hal.h"
#include "log/logger.h"
#include <stdint.h>
#include <string.h>

eSTATUS_t DeviceInitAll (BoardConf_t* pBoardConf);
eSTATUS_t DeviceStartAll (void);

#endif // DEVICE_DEVICE_H