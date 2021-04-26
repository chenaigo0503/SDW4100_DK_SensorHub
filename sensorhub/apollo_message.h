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
#include <stdint.h>

#ifndef APOLLO_MESSAGE_H
#define APOLLO_MESSAGE_H

#define APOLLO_MSG_MAX              1024

#define APOLLO_MESSAGE_HEAD         0xAA
#define APOLLO_HUB_PID              0x01

#define APOLLO_HEART_BEAT_CMD       0x00
#define APOLLO_HEART_BEAT_RESP      0x01
#define APOLLO_GET_VERSION_CMD      0x02
#define APOLLO_GET_VERSION_RESP     0x03
#define APOLLO_FW_UPDATA_CMD        0x04
#define APOLLO_FW_UPDATA_RESP       0x05
#define APOLLO_FW_UPDATA_DATA       0x06
#define APOLLO_FW_DATA_RESP         0x07
#define APOLLO_FW_DATAEND_CMD       0x08
#define APOLLO_FW_DATAEND_RESP      0x09
#define APOLLO_SET_DATE_TIME_CMD    0x0A    /* 0x0A: set Date and Time information to Sensor Hub request command */
#define APOLLO_SET_DATE_TIME_RESP   0x0B    /* 0x0B: set Date and TIme information to Sensor Hub response command */
#define APOLLO_ACC_CALI_CMD         0x14
#define APOLLO_ACC_CALI_RESP        0x15
#define APOLLO_SET_ACC_CALI_CMD     0x16
#define APOLLO_SET_ACC_CALI_RESP    0x17
#define APOLLO_SET_GNSS_CMD         0x18
#define APOLLO_SET_GNSS_RESP        0x19

enum mid_def {
    APOLLO_SENSOR_CONTROL_CMD = 0x1A,           /* 0x1A: set sensor control request command */
    APOLLO_SENSOR_CONTROL_RESP,                 /* 0x1B: set sensor control response  */
};

enum SensorType {
    Accelerometer = 0x00,   /* 0x00: Accelerometer Sensor */
    Gyroscope,              /* 0x01: Gyroscope Sensor */
    Temperature,            /* 0x02: Temperature Sensor */
    Pressure,               /* 0x03: Pressure Sensor */
    Magnetometer,           /* 0x04: Magnetometer Sensor */
    HeartRate,              /* 0x05: Heart Rate Sensor */
    StepCounter,            /* 0x06: Step Counter Sensor*/
    WristTilt,              /* 0x07: Wrist Tilt Sensor */
    OffBodyDetect,          /* 0x08: Off Body Detect Sensor */
    maxsensortype,
};

enum SensorControl_Command {
    SENSOR_DISABLE = 0x00,  /* 0x00: Sensor Disable */
    SENSOR_ENABLE,          /* 0x01: Sensor Enable */
};

#define APOLLO_SENSOR_0_EVNT        0xC0
#define APOLLO_SENSOR_1_EVNT        0xC1
#define APOLLO_SENSOR_2_EVNT        0xC2
#define APOLLO_SENSOR_3_EVNT        0xC3
#define APOLLO_SENSOR_4_EVNT        0xC4
#define APOLLO_SENSOR_5_EVNT        0xC5
#define APOLLO_SENSOR_6_EVNT        0xC6
#define APOLLO_SENSOR_7_EVNT        0xC7
#define APOLLO_SENSOR_8_EVNT        0xC8

typedef enum APOLLO_HUB_RESP_RESULT {
    APOLLO_HUB_RESP_SUCCESS = 0x00,         /* 0x00: response success */
    APOLLO_HUB_RESP_FAIL,                   /* 0x01: response fail */
    APOLLO_HUB_RESP_ERROR_LENGTH,           /* 0x02: response wrong data length */
    APOLLO_HUB_RESP_ERROR_CRC,              /* 0x03: response wrong CRC validation */
    APOLLO_HUB_RESP_ERROR_FS_PID,           /* 0x04: response wrong FS or PID */
    APOLLO_HUB_RESP_ERROR_PB_DECODE,        /* 0x05: response wrong nanopb decode */
    APOLLO_HUB_RESP_ERROR_DATE_INFO,        /* 0x06: response wrong date information */
    APOLLO_HUB_RESP_ERROR_TIME_INFO,        /* 0x07: response wrong time information */
    APOLLO_HUB_RESP_ERROR_MALLOC,           /* 0x08: response wrong malloc */
} APOLLO_HUB_RESP_RESULT_t;

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

// function
int unpack_data(uint8_t* message_pack);
uint8_t send_resp_msg(uint8_t msg_id, uint8_t* msg_data);
uint8_t send_resp_msg_to_host(uint8_t msg_id, uint16_t length, uint8_t* msg_data);
uint8_t send_event_msg(uint8_t msg_id, uint8_t* msg_data);
uint8_t send_event_msg_to_host(uint8_t msg_id, uint16_t msg_length, uint8_t* msg_data);
int sensor_event_enquene(uint8_t mid, uint8_t* sns_data, uint16_t sns_len);
void msg_dequene(void);

#endif // APOLLO_MESSAGE_H
