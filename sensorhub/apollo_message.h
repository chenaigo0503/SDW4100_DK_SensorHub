//*****************************************************************************
//
//! @file apollo_message.h
//!
//! @brief apollo use as sensor Hub.
//!
//! @addtogroup sensor Hub used message lib.
//! @ingroup sensorhub
//! @{
//
//*****************************************************************************

#ifndef APOLLO_MESSAGE_H
#define APOLLO_MESSAGE_H

#define APOLLO_MSG_MAX             1024

#define APOLLO_MESSAGE_HEAD        0xaa
#define APOLLO_HUB_PID             0x01

#define APOLLO_HEART_BEAT_CMD      0x00
#define APOLLO_HEART_BEAT_RESP     0x01
#define APOLLO_GET_VERSION_CMD     0x02
#define APOLLO_GET_VERSION_RESP    0x03
#define APOLLO_FW_UPDATA_CMD       0x04
#define APOLLO_FW_UPDATA_RESP      0x05
#define APOLLO_FW_UPDATA_DATA      0x06
#define APOLLO_FW_DATA_RESP        0x07
#define APOLLO_FW_DATAEND_CMD      0x08
#define APOLLO_FW_DATAEND_RESP     0x09

#define APOLLO_SET_DATE_CMD        0x10
#define APOLLO_SET_DATE_RESP       0x11
#define APOLLO_SET_TIME_CMD        0x12
#define APOLLO_SET_TIME_RESP       0x13
#define APOLLO_ACC_CALI_CMD        0x14
#define APOLLO_ACC_CALI_RESP       0x15
#define APOLLO_SET_ACC_CALI_CMD    0x16
#define APOLLO_SET_ACC_CALI_RESP   0x17

#define APOLLO_SENSOR_0_STOP_CMD   0x40     // ACC
#define APOLLO_SENSOR_1_STOP_CMD   0x41     // GYRO
#define APOLLO_SENSOR_2_STOP_CMD   0x42     // TEMP
#define APOLLO_SENSOR_3_STOP_CMD   0x43     // presuer
#define APOLLO_SENSOR_4_STOP_CMD   0x44     // compass
#define APOLLO_SENSOR_5_STOP_CMD   0x45     // Heart Rate
#define APOLLO_SENSOR_6_STOP_CMD   0x46     // Step Detec
#define APOLLO_SENSOR_0_START_CMD  0x60     // 
#define APOLLO_SENSOR_1_START_CMD  0x61
#define APOLLO_SENSOR_2_START_CMD  0x62
#define APOLLO_SENSOR_3_START_CMD  0x63
#define APOLLO_SENSOR_4_START_CMD  0x64
#define APOLLO_SENSOR_5_START_CMD  0x65
#define APOLLO_SENSOR_6_START_CMD  0x66
#define APOLLO_SENSOR_0_STOP_RESP  0x80
#define APOLLO_SENSOR_1_STOP_RESP  0x81
#define APOLLO_SENSOR_2_STOP_RESP  0x82
#define APOLLO_SENSOR_3_STOP_RESP  0x83
#define APOLLO_SENSOR_4_STOP_RESP  0x84
#define APOLLO_SENSOR_5_STOP_RESP  0x85
#define APOLLO_SENSOR_6_STOP_RESP  0x86
#define APOLLO_SENSOR_0_START_RESP 0xa0
#define APOLLO_SENSOR_1_START_RESP 0xa1
#define APOLLO_SENSOR_2_START_RESP 0xa2
#define APOLLO_SENSOR_3_START_RESP 0xa3
#define APOLLO_SENSOR_4_START_RESP 0xa4
#define APOLLO_SENSOR_5_START_RESP 0xa5
#define APOLLO_SENSOR_6_START_RESP 0xa6
#define APOLLO_SENSOR_0_EVNT       0xc0
#define APOLLO_SENSOR_1_EVNT       0xc1
#define APOLLO_SENSOR_2_EVNT       0xc2
#define APOLLO_SENSOR_3_EVNT       0xc3
#define APOLLO_SENSOR_4_EVNT       0xc4
#define APOLLO_SENSOR_5_EVNT       0xc5
#define APOLLO_SENSOR_6_EVNT       0xc6

typedef struct apollo_msg{
    uint8_t mid;
    uint8_t len;
    uint8_t* data;
    struct apollo_msg* next;
}apollo_msg;

typedef struct msg_link{
    apollo_msg* front;
    apollo_msg* rear;
}msg_link;

static uint8_t apollo_message_len[256] = {
//  x0  x1  x2 x3 x4 x5 x6 x7 x8 x9 xA xB xC xD xE xF
    0,  0,  0, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, //0x
    4,  0,  3, 0, 0, 3, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, //1x
    0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, //2x
    0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, //3x
    0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, //4x
    0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, //5x
    0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, //6x
    0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, //7x
    0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, //8x
    0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, //9x
    0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, //Ax
    0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, //Bx
    12, 12, 4, 4, 6, 10, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, //Cx
    0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, //Dx
    0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, //Ex
    0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, //Fx
};

// function
int unpack_data(uint8_t* message_pack);
uint8_t send_resp_msg(uint8_t msg_id);
uint8_t send_event_msg(uint8_t msg_id, uint8_t* msg_data);
uint8_t send_data_msg(uint8_t msg_id, uint8_t* msg_data);
int sensor_event_enquene(uint8_t mid, uint8_t* sns_data, uint16_t sns_len);
void msg_dequene(void);

#endif // APOLLO_MESSAGE_H
