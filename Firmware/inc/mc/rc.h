#ifndef CONTROL_RC_H
#define CONTROL_RC_H

#include "core/core.h"

#define RC_MAX_CHANNELS 16U
#define RC_CHANNEL_MIN 1000U
#define RC_CHANNEL_MID 1500U
#define RC_CHANNEL_MAX 2000U

#define RC_CHANNEL_IDX_ROLL     0U
#define RC_CHANNEL_IDX_PITCH    1U
#define RC_CHANNEL_IDX_YAW      2U
#define RC_CHANNEL_IDX_THROTTLE 3U
#define RC_CHANNEL_IDX_AUX_1    4U
#define RC_CHANNEL_IDX_AUX_2    5U

eSTATUS_t Rc_Init(void);
eSTATUS_t Rc_Update(void);

#endif // CONTROL_RC_H
