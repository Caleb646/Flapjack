// Generated with umsg_gen - do not edit by hand
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <umsg_types.h>

// msg structure typedefs

typedef enum
{
    EMISSION_MODE_ALTITUDE_HOLD,
    EMISSION_MODE_POSITION_HOLD,
    EMISSION_MODE_WAYPOINT,
    EMISSION_MODE_RTL
} umsg_mission_mode_t;

typedef struct
{
    umsg_mission_mode_t mode;
    float target_pos[3];
    float target_heading;
    uint8_t armed;
} umsg_mission_state_t;

// api function headers
umsg_sub_handle_t umsg_mission_state_subscribe(uint32_t prescaler, uint8_t length);
umsg_sub_handle_t umsg_mission_state_subscribe_ch(uint32_t prescaler, uint8_t length, uint8_t channel);
void umsg_mission_state_publish(umsg_mission_state_t* data);
void umsg_mission_state_publish_ch(umsg_mission_state_t* data, uint8_t channel);
uint8_t umsg_mission_state_receive(umsg_sub_handle_t queue, umsg_mission_state_t* data, uint32_t timeout);
// Running total of publishes; wraps, so diff two reads as unsigned. Only exact
// while the topic has a single publisher (one task, no ISR): see umsg_publish.
uint32_t umsg_mission_state_count(void);


#ifdef __cplusplus
}
#endif

