//*****************************************************************************
//
//! @file apollo_message.h
//!
//! @brief apollo use as sensor Hub.
//!
//! @addtogroup sensor Hub used message lib.
//! @ingroup sensorhub
//
//*****************************************************************************

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "apollo_message.h"
#include "crc8.h"
#include "am_mcu_apollo.h"
#include "am_util.h"

static void *g_pIOSHandle;
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
    apollo_msg* front_msg = NULL;

    if(msg_link_quene.front == NULL)
        return;

    msg_link_quene.front = msg_link_quene.front->next;
}

int unpack_data(uint8_t* message_pack)
{
    apollo_msg* ap_msg = NULL;

    ap_msg = msg_malloc(sizeof(apollo_msg));

    if(message_pack[0] == APOLLO_MESSAGE_HEAD && message_pack[1] ==APOLLO_HUB_PID)
    {
        ap_msg->mid = message_pack[2];
        ap_msg->len = *(uint16_t*)(&message_pack[3]);

        if(ap_msg->len != apollo_message_len[ap_msg->mid])
        {
            return -1;
        }
    } else {
        return -1;
    }

    if(message_pack[ap_msg->len + 5] == CalcCrc8(message_pack, ap_msg->len + 5))
    {
        ap_msg->data = msg_malloc(ap_msg->len);
        if(ap_msg->data == NULL)
            return -1;

        memcpy(ap_msg->data, &message_pack[5], ap_msg->len);
        msg_enquene(ap_msg);

        return 0;
    } else {
        return -1;
    }
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

uint8_t send_resp_msg(uint8_t msg_id)
{
    uint8_t send_msg[6] = {0xAA, 0x01};
    uint32_t num_write;
    if(msg_id == APOLLO_GET_VERSION_CMD)
    {
        uint8_t ver_msg[9] = {0xAA, 0x01};
        ver_msg[2] = msg_id + 1;
        ver_msg[3] = 3;
        ver_msg[4] = 0;

        //get version
        //ver_msg[5],ver_msg[6],ver_msg[7]
        ver_msg[8] = CalcCrc8(ver_msg, sizeof(ver_msg) - 1);
        am_hal_ios_fifo_write(g_pIOSHandle, ver_msg, sizeof(ver_msg), &num_write);
        if(sizeof(ver_msg) < num_write)
            return -1;
        return 0;
    }

    send_msg[3] = 0;
    send_msg[4] = 0;
    if(msg_id < 0x40)
    {
        send_msg[2] = msg_id + 1;
    }
    else if(msg_id >= 0x40 && msg_id < 0x80)
    {
        send_msg[2] = msg_id + 0x40;
    }

    send_msg[5] = CalcCrc8(send_msg, sizeof(send_msg) - 1);
    am_hal_ios_fifo_write(g_pIOSHandle, send_msg, sizeof(send_msg), &num_write);
    if(sizeof(send_msg) < num_write)
        return -1;

    return 0;
}

/*static uint8_t response_sys_cmd(uint8_t system_cmd)
{
    switch(system_cmd)
    {
        case APOLLO_HEART_BEAT_CMD:
            response_heart_beat();
            break;

        case APOLLO_GET_VERSION_CMD:
            response_version_info();
            break;

        case APOLLO_FW_UPDATA_CMD:
            switch_to_updata_mode();
            break;

        default:
            return -EVNOT;
    }

    return 0;
}

static uint8_t response_set_cmd(uint8_t setting_cmd)
{
    switch(setting_cmd)
    {
        case APOLLO_SET_DATE_CMD:
        {
            void m_data[3];

            unpack_data(uint8_t message_id, uint8_t* message_pack, void* data_pack);
        }
    }
}
*/
