// Generated with umsg_gen on 2026-08-15
#include <umsg.h>
#include <umsg_mission.h>

// msg instances
static umsg_msg_metadata_t msg_mission_state = {.name = "mission_state"};

// msg api's
// mission_state
umsg_sub_handle_t umsg_mission_state_subscribe(uint32_t prescaler, uint8_t length)
{
    return umsg_subscribe(&msg_mission_state, prescaler, sizeof(umsg_mission_state_t), length, 0);
}
umsg_sub_handle_t umsg_mission_state_subscribe_ch(uint32_t prescaler, uint8_t length, uint8_t channel)
{
    return umsg_subscribe(&msg_mission_state, prescaler, sizeof(umsg_mission_state_t), length, channel);
}
void umsg_mission_state_publish(umsg_mission_state_t* data)
{
    umsg_publish(&msg_mission_state, data, 0);
}
void umsg_mission_state_publish_ch(umsg_mission_state_t* data, uint8_t channel)
{
    umsg_publish(&msg_mission_state, data, channel);
}
uint8_t umsg_mission_state_receive(umsg_sub_handle_t queue, umsg_mission_state_t* data, uint32_t timeout)
{
    return umsg_receive(queue, data, timeout);
}
uint8_t umsg_mission_state_peek(umsg_mission_state_t* data)
{
    return umsg_peek(&msg_mission_state, data, sizeof(umsg_mission_state_t));
}

