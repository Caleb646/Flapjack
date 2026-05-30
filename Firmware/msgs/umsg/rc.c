// Generated with umsg_gen on 2026-05-30
#include <umsg.h>
#include <umsg_rc.h>

// msg instances
static umsg_msg_metadata_t msg_rc_input = {.name = "rc_input"};

// msg api's
// rc_input
umsg_sub_handle_t umsg_rc_input_subscribe(uint32_t prescaler, uint8_t length)
{
    return umsg_subscribe(&msg_rc_input, prescaler, sizeof(umsg_rc_input_t), length, 0);
}
umsg_sub_handle_t umsg_rc_input_subscribe_ch(uint32_t prescaler, uint8_t length, uint8_t channel)
{
    return umsg_subscribe(&msg_rc_input, prescaler, sizeof(umsg_rc_input_t), length, channel);
}
void umsg_rc_input_publish(umsg_rc_input_t* data)
{
    umsg_publish(&msg_rc_input, data, 0);
}
void umsg_rc_input_publish_ch(umsg_rc_input_t* data, uint8_t channel)
{
    umsg_publish(&msg_rc_input, data, channel);
}
uint8_t umsg_rc_input_receive(umsg_sub_handle_t queue, umsg_rc_input_t* data, uint32_t timeout)
{
    return umsg_receive(queue, data, timeout);
}
uint8_t umsg_rc_input_peek(umsg_rc_input_t* data)
{
    return umsg_peek(&msg_rc_input, data, sizeof(umsg_rc_input_t));
}

