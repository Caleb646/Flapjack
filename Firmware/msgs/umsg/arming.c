// Generated with umsg_gen on 2026-08-15
#include <umsg.h>
#include <umsg_arming.h>

// msg instances
static umsg_msg_metadata_t msg_arming_request = {.name = "arming_request"};

// msg api's
// arming_request
umsg_sub_handle_t umsg_arming_request_subscribe(uint32_t prescaler, uint8_t length)
{
    return umsg_subscribe(&msg_arming_request, prescaler, sizeof(umsg_arming_request_t), length, 0);
}
umsg_sub_handle_t umsg_arming_request_subscribe_ch(uint32_t prescaler, uint8_t length, uint8_t channel)
{
    return umsg_subscribe(&msg_arming_request, prescaler, sizeof(umsg_arming_request_t), length, channel);
}
void umsg_arming_request_publish(umsg_arming_request_t* data)
{
    umsg_publish(&msg_arming_request, data, 0);
}
void umsg_arming_request_publish_ch(umsg_arming_request_t* data, uint8_t channel)
{
    umsg_publish(&msg_arming_request, data, channel);
}
uint8_t umsg_arming_request_receive(umsg_sub_handle_t queue, umsg_arming_request_t* data, uint32_t timeout)
{
    return umsg_receive(queue, data, timeout);
}
