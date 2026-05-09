#ifndef FC_RC_H
#define FC_RC_H

#include <stdbool.h>
#include <stdint.h>

#include "core/core.h"

#define RC_CHANNEL_MIN          1000U
#define RC_CHANNEL_MID          1500U
#define RC_CHANNEL_MAX          2000U
#define RC_MAX_CHANNELS         16U

#define RC_CHANNEL_IDX_THROTTLE 0U
#define RC_CHANNEL_IDX_YAW      1U
#define RC_CHANNEL_IDX_PITCH    2U
#define RC_CHANNEL_IDX_ROLL     3U
#define RC_CHANNEL_IDX_AUX_1    4U
#define RC_CHANNEL_IDX_AUX_2    5U
#define RC_CHANNEL_IDX_AUX_3    6U
#define RC_CHANNEL_IDX_AUX_4    7U

eSTATUS_t Rc_Init (void);
eSTATUS_t Rc_Update (uint32_t usCurrentTime, uint32_t usDeltaTime);


#endif /* FC_RC_H */