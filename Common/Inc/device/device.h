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

#define DEVICE_INIT_ALL(pSTATUS, pBOARD_CONF)     \
    do {                                          \
        *(pSTATUS) = DeviceInitAll (pBOARD_CONF); \
    } while (0)

#endif // DEVICE_DEVICE_H