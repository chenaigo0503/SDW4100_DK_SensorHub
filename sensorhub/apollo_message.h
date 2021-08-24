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

#include <stdint.h>
#include "crc8.h"
#include "am_mcu_apollo.h"
#include "am_util.h"
#include "apollo3_init.h"
#include "apollo_tracelog.h"
#include <stddef.h>
#include <string.h>

#define APOLLO_MSG_MAX              1024

#define APOLLO_MESSAGE_HEAD         0xAA
#define APOLLO_HUB_PID              0x01

enum mid_def {
    APOLLO_HEART_BEAT_CMD = 0x00,       /* 0x00: heart beat request command */
    APOLLO_HEART_BEAT_RESP,             /* 0x01: heart beat resonse command */
    APOLLO_GET_VERSION_CMD,             /* 0x02: get version request command */
    APOLLO_GET_VERSION_RESP,            /* 0x03: get version response command */
    APOLLO_FW_UPDATA_CMD,               /* 0x04: update firmware request command */
    APOLLO_FW_UPDATA_RESP,              /* 0x05: update firmware response comannd */
    APOLLO_FW_UPDATA_DATA,              /* 0x06: update firmware data request command */
    APOLLO_FW_DATA_RESP = 0x07,         /* 0x07: update firmware data response command */
    APOLLO_FW_DATAEND_CMD,              /* 0x08: update firmware data end request command */
    APOLLO_FW_DATAEND_RESP,             /* 0x09: update firmware data end response command */
    APOLLO_SET_DATE_TIME_CMD,           /* 0x0A: set Date and Time information to Sensor Hub request command */
    APOLLO_SET_DATE_TIME_RESP,          /* 0x0B: set Date and TIme information to Sensor Hub response command */
    APOLLO_SET_STEPCOUNTER_RESET_CMD,   /* 0x0C: set step counter reset request command */
    APOLLO_SET_STEPCOUNTER_RESET_RESP,  /* 0x0D: set step counter reset response command */
    APOLLO_ACC_CALI_CMD = 0x14,         /* 0x14: accel cali request command */
    APOLLO_ACC_CALI_RESP,               /* 0x15: accel cali response command */
    APOLLO_SET_ACC_CALI_CMD,            /* 0x16: set accel cali request command */
    APOLLO_SET_ACC_CALI_RESP,           /* 0x17: set accel cali response command */
    APOLLO_SET_GNSS_CMD,                /* 0x18: set GNSS information to Sensor Hub request command */
    APOLLO_SET_GNSS_RESP,               /* 0x19: set GNSS information to Sensor Hub response command */

    /* APOLLO_SENSOR_CONTROL_CMD
    --------------------------------------
    | DATA                               |
    --------------------------------------
    | Byte4      | Byte5                 |
    --------------------------------------
    | SensorType | SensorControl_Command |
    --------------------------------------*/
    APOLLO_SENSOR_CONTROL_CMD,          /* 0x1A: set sensor control request command */
    APOLLO_SENSOR_CONTROL_RESP,         /* 0x1B: set sensor control response */

    APOLLO_MID_MAX_NUMBER,
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

enum sensor_type {
    APOLLO_SENSOR_0_EVNT = 0xC0, /* 0xC0: Accelerometer sensor data event */
    APOLLO_SENSOR_1_EVNT,        /* 0xC1: Gyroscope sensor data event */
    APOLLO_SENSOR_2_EVNT,        /* 0xC2: BMP280 Temperature sensor data event */
    APOLLO_SENSOR_3_EVNT,        /* 0xC3: BMP280 Pressure sensor data event */
    APOLLO_SENSOR_4_EVNT,        /* 0xC4: Magentic sensor data event  */
    APOLLO_SENSOR_5_EVNT,        /* 0xC5: Heart Rate sensor data event */
    APOLLO_SENSOR_6_EVNT,        /* 0xC6: Step Counter sensor data event */
    APOLLO_SENSOR_7_EVNT,        /* 0xC7: A+G Wrist sensor data event */
    APOLLO_SENSOR_8_EVNT,        /* 0xC8: PAH8011 off body detect sensor data event */
};

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
uint8_t send_resp_msg_to_host(uint8_t msg_id, uint8_t length, uint8_t* msg_data);
uint8_t send_event_msg_to_host(uint8_t msg_id, uint8_t msg_length, uint8_t* msg_data);
int sensor_event_enquene(uint8_t mid, uint8_t* sns_data, uint16_t sns_len);
void msg_dequene(void);

#endif // APOLLO_MESSAGE_H
