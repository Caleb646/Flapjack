// Generated with umsg_gen - do not edit by hand
#include <umsg.h>
#include <umsg_control.h>

// msg instances
static umsg_msg_metadata_t msg_control_status = {.name = "control_status"};

// msg api's
// control_status
umsg_sub_handle_t umsg_control_status_subscribe(uint32_t prescaler, uint8_t length)
{
    return umsg_subscribe(&msg_control_status, prescaler, sizeof(umsg_control_status_t), length, 0);
}
umsg_sub_handle_t umsg_control_status_subscribe_ch(uint32_t prescaler, uint8_t length, uint8_t channel)
{
    return umsg_subscribe(&msg_control_status, prescaler, sizeof(umsg_control_status_t), length, channel);
}
void umsg_control_status_publish(umsg_control_status_t* data)
{
    umsg_publish(&msg_control_status, data, 0);
}
void umsg_control_status_publish_ch(umsg_control_status_t* data, uint8_t channel)
{
    umsg_publish(&msg_control_status, data, channel);
}
uint8_t umsg_control_status_receive(umsg_sub_handle_t queue, umsg_control_status_t* data, uint32_t timeout)
{
    return umsg_receive(queue, data, timeout);
}
uint32_t umsg_control_status_count(void)
{
    return msg_control_status.counter;
}
