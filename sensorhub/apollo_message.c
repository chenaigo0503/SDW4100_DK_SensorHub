//*****************************************************************************
//
//! @file apollo_message.c
//!
//! @brief apollo use as sensor Hub.
//!
//! @addtogroup sensor Hub used message lib.
//! @ingroup sensorhub
//
//*****************************************************************************

#include "apollo_message.h"

extern void *g_pIOSHandle;
static uint8_t msg_buf[APOLLO_MSG_MAX];
static uint8_t* p_msg_buf = msg_buf;

volatile msg_link msg_link_quene;

void* msg_malloc(size_t size)
{
    void* msg_addr = NULL;
    if(size > APOLLO_MSG_MAX)
        return NULL;

    if(size > (msg_buf + APOLLO_MSG_MAX - p_msg_buf))
        p_msg_buf = msg_buf;

    msg_addr = p_msg_buf;
    p_msg_buf += size;
    memset(msg_addr, 0, size);

    return msg_addr;
}

void msg_enquene(apollo_msg* ap_msg)
{
    if(msg_link_quene.front == NULL)
    {
        msg_link_quene.rear = NULL;
        msg_link_quene.front = ap_msg;
        return;
    }

    if(msg_link_quene.rear == NULL)
        msg_link_quene.rear = msg_link_quene.front;

    msg_link_quene.rear->next = ap_msg;
    msg_link_quene.rear = ap_msg;
}

void msg_dequene(void)
{
    if(msg_link_quene.front == NULL)
        return;

    msg_link_quene.front = msg_link_quene.front->next;
}

int unpack_data(uint8_t* message_pack)
{
    apollo_msg* ap_msg = NULL;
    uint8_t ret = APOLLO_HUB_RESP_SUCCESS;

    ap_msg = msg_malloc(sizeof(apollo_msg));

    if ((message_pack[0] != APOLLO_MESSAGE_HEAD) || (message_pack[1] != APOLLO_HUB_PID))
    {
        // Does not conform to transport protocol
        return -1;
    }

    ap_msg->mid = message_pack[2];
    ap_msg->len = message_pack[3];

    if (message_pack[ap_msg->len + 4] != CalcCrc8(message_pack, ap_msg->len + 4))
    {
        // CRC 8 check failed
        ret = APOLLO_HUB_RESP_ERROR_CRC;
        send_resp_msg_to_host(ap_msg->mid, 1, &ret);
        return -3;
    }

    ap_msg->data = msg_malloc(ap_msg->len);
    if (ap_msg->data == NULL)
    {
        ret = APOLLO_HUB_RESP_ERROR_MALLOC;
        send_resp_msg_to_host(ap_msg->mid, 1, &ret);
        return -1;
    }

    memcpy(ap_msg->data, &message_pack[4], ap_msg->len);
    msg_enquene(ap_msg);

    return 0;
}

int sensor_event_enquene(uint8_t mid, uint8_t* sns_data, uint16_t sns_len)
{
    apollo_msg* ap_msg = NULL;

    ap_msg = msg_malloc(sizeof(apollo_msg));
    ap_msg->mid = mid;
    ap_msg->len = sns_len;
    ap_msg->data = msg_malloc(ap_msg->len);
    if(ap_msg->data == NULL)
        return -1;

    memcpy(ap_msg->data, sns_data, ap_msg->len);
    msg_enquene(ap_msg);

    return 0;
}

uint8_t send_resp_msg_to_host(uint8_t msg_id, uint8_t msg_length, uint8_t* msg_data)
{
    uint8_t send_msg[128] = { 0x00 };
    uint32_t num_write;
    uint32_t iosUsedSpace;

    send_msg[0] = APOLLO_MESSAGE_HEAD;
    send_msg[1] = APOLLO_HUB_PID;
    send_msg[2] = msg_id;

    if (msg_id < 0x40)
    {
        send_msg[2] = msg_id + 1;
    }
    else if ((msg_id >= 0x40) && (msg_id < 0x80))
    {
        send_msg[2] = msg_id + 0x40;
    }
    else
    {
        return 1;
    }

    send_msg[3] = msg_length;

    if (send_msg[3] != 0)
    {
        if (msg_data == NULL)
        {
            return 2;
        }

        memcpy(&send_msg[4], msg_data, send_msg[3]);
    }

    send_msg[send_msg[3] + 4] = CalcCrc8(send_msg, send_msg[3] + 4);

    am_hal_ios_fifo_space_used(g_pIOSHandle, &iosUsedSpace);
    while (iosUsedSpace)
    {
        am_util_delay_ms(5);
        am_hal_ios_fifo_space_used(g_pIOSHandle, &iosUsedSpace);
        if (iosUsedSpace)
            inform_host();
    }

    am_hal_ios_fifo_write(g_pIOSHandle, send_msg, send_msg[3] + 5, &num_write);

    if(sizeof(send_msg) < num_write)
        return 3;

    am_hal_ios_control(g_pIOSHandle, AM_HAL_IOS_REQ_FIFO_UPDATE_CTR, NULL);
    wait_fifo_empty();

    return 0;
}

uint8_t send_event_msg_to_host(uint8_t msg_id, uint8_t msg_length, uint8_t* msg_data)
{
    uint8_t send_msg[64] = { 0x00 };
    uint32_t num_write;
    uint32_t iosUsedSpace;

    if (msg_id < 0xc0 || msg_id > 0xe0)
    {
        return 1;
    }

    send_msg[0] = APOLLO_MESSAGE_HEAD;
    send_msg[1] = APOLLO_HUB_PID;
    send_msg[2] = msg_id;
    send_msg[3] = msg_length;

    if (send_msg[3] != 0)
    {
        if (msg_data == NULL)
        {
            return 2;
        }

        memcpy(&send_msg[4], msg_data, send_msg[3]);
    }

    send_msg[send_msg[3] + 4] = CalcCrc8(send_msg, send_msg[3] + 4);

    am_hal_ios_fifo_space_used(g_pIOSHandle, &iosUsedSpace);
    while (iosUsedSpace)
    {
        am_util_delay_ms(5);
        am_hal_ios_fifo_space_used(g_pIOSHandle, &iosUsedSpace);
        if (iosUsedSpace)
            inform_host();
    }

    am_hal_ios_fifo_write(g_pIOSHandle, send_msg, send_msg[3] + 5, &num_write);
    if(sizeof(send_msg) < num_write)
    {
        return 3;
    }

    return 0;
}
