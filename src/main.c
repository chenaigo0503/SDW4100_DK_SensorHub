//*****************************************************************************
//
//! @file main.c
//!
//! @brief A simple "Hello World" example using the UART peripheral.
//!
//! Purpose: This example prints a "Hello World" message with some device info
//! over UART at 115200 baud. To see the output of this program, run AMFlash,
//! and configure the console for UART. The example sleeps after it is done
//! printing.
//
//*****************************************************************************

//*****************************************************************************
//
// Copyright (c) 2020, Thundercomm, Inc.
// All rights reserved.
//
// This is part of revision 2.5.1 of the AmbiqSuite Development Package.
//
//*****************************************************************************

#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"

#include "apollo_tracelog.h"
#include "apollo3_init.h"
#include "apollo3_uart.h"
#include "apollo3_amotas.h"
#include "apollo3_ios.h"
#include "apollo3_rtc.h"
#include "apollo_delay2run.h"
#include "apollo_message.h"

#include "lsm6dso_hal.h"
#include "bmp280_hal.h"
#include "ak09918_hal.h"
#include "pah8011_hal.h"

#include "pb_decode.h"
#include "pb_encode.h"
#include "UserInformation.pb.h"

extern volatile msg_link msg_link_quene;
extern uint8_t amotaStart;
extern void factory_test(void);

//*****************************************************************************
//
// Main
//
//*****************************************************************************
int main(void)
{
    APOLLO_HUB_RESP_RESULT_t status = APOLLO_HUB_RESP_SUCCESS;
    apollo3_init();
    rtc_init();
    get_version();

#if (APOLLO_LOG_LEVEL)
#if (APOLLO3_HUB_VER == 1)
    // uart to print log
    apollo3_uart_init();
    am_util_stdio_printf_init(uart_print);
#else
    // swo to print log
    am_bsp_itm_printf_enable();
#endif
#endif

    ios_init();
    lsm6dso_init();
    ak099xx_init();
    bmp280_init();
    pah_init();

    am_hal_interrupt_master_enable();

    // Print the sw infomation.
    END_LINE;
    PR_INFO("Apollo 3 Blue for Sensor Hub, SW ver:%02x.%02x.%02x ", sw_version[0], sw_version[1], sw_version[2]);
    PR_INFO("Build on %s at %s.", __DATE__, __TIME__);

    // init amotas
    dump_ota_status();

    while(1)
    {
        factory_test();

        if (msg_link_quene.front != NULL)
        {
            PR_DBG("Recive msg queue mid: 0x%02x", msg_link_quene.front->mid);

            switch (msg_link_quene.front->mid)
            {
                case APOLLO_GET_VERSION_CMD:
                    send_resp_msg_to_host(msg_link_quene.front->mid, 3, sw_version);
                    break;

                case APOLLO_FW_UPDATA_CMD:
                    amotas_init();
                    send_resp_msg_to_host(msg_link_quene.front->mid, 0, NULL);
                    break;

                case APOLLO_FW_UPDATA_DATA:
                    distribute_pack(msg_link_quene.front->len, msg_link_quene.front->data, 0);
                    send_resp_msg_to_host(msg_link_quene.front->mid, 0, NULL);
                    break;

                case APOLLO_FW_DATAEND_CMD:
                    distribute_pack(0, NULL, 1);
                    send_resp_msg_to_host(msg_link_quene.front->mid, 0, NULL);
                    break;

                case APOLLO_ACC_CALI_CMD:
                {
                    uint8_t ret;
                    int8_t acc_cail[3] = {0};
                    ret = lsm6dso_acc_cali();
                    if (ret)
                    {
                        acc_cail[0] = 0xFF;

                        send_resp_msg_to_host(msg_link_quene.front->mid, 3, (uint8_t *)(&acc_cail[0]));
                        break;
                    }

                    lsm6dso_get_acc_cali_data((uint8_t*)acc_cail);

                    send_resp_msg_to_host(msg_link_quene.front->mid, 3, (uint8_t *)(&acc_cail[0]));

                    break;
                }

                case APOLLO_SET_ACC_CALI_CMD:
                    PR_INFO("set cali accel command");
                    lsm6dso_set_acc_cali_data(msg_link_quene.front->data);

                    send_resp_msg_to_host(msg_link_quene.front->mid, 0, NULL);
                    break;

                case APOLLO_SET_GNSS_CMD:
                {
                    PR_INFO("set GNSS command");

                    GNSSInformation userInfo;

                    pb_istream_t dec_stream;
                    dec_stream = pb_istream_from_buffer(msg_link_quene.front->data, msg_link_quene.front->len);
                    if (!pb_decode(&dec_stream, GNSSInformation_fields, &userInfo))
                    {
                        PR_INFO("pb decode error in %s\n", __func__);
                        return -1;
                    }

                    PR_INFO("Unpack: %f, %f\n", userInfo.latitude, userInfo.longitude);

                    if (userInfo.latitude == 11.0)
                    {
                        send_resp_msg_to_host(msg_link_quene.front->mid, 1, (uint8_t *)&status);
                    }
                    else
                    {
                        status = APOLLO_HUB_RESP_FAIL;
                        send_resp_msg_to_host(msg_link_quene.front->mid, 1, (uint8_t *)&status);
                    }
                    break;
                }

                case APOLLO_SET_DATE_TIME_CMD:
                {
                    PR_INFO("set APOLLO_SET_DATE_TIME_CMD command");

                    DateTimeInformation userInfo;

                    pb_istream_t dec_stream;
                    dec_stream = pb_istream_from_buffer(msg_link_quene.front->data, msg_link_quene.front->len);
                    if (!pb_decode(&dec_stream, DateTimeInformation_fields, &userInfo))
                    {
                        PR_ERR("pb decode error in %s\n", __func__);
                        status = APOLLO_HUB_RESP_ERROR_PB_DECODE;
                        send_resp_msg_to_host(msg_link_quene.front->mid, 1, (uint8_t *)&status);
                        return -1;
                    }

                    uint16_t set_year = (uint16_t)(userInfo.year);
                    uint8_t set_month = (uint8_t)(userInfo.month);
                    uint8_t set_day = (uint8_t)(userInfo.day);
                    uint8_t set_hour = (uint8_t)(userInfo.hour);
                    uint8_t set_minute = (uint8_t)(userInfo.minute);
                    uint8_t set_sceond = (uint8_t)(userInfo.second);

                    PR_INFO("SET date: %04d-%02d-%02d", set_year, set_month, set_day);
                    PR_INFO("SET time: %02d:%02d:%02d", set_hour, set_minute, set_sceond);

                    if ((1899 > set_year) || (set_year > 2199) || (set_month > 12) || (set_day > 31))
                    {
                        PR_ERR("Invalid setting date parameter");

                        status = APOLLO_HUB_RESP_ERROR_DATE_INFO;
                        send_resp_msg_to_host(msg_link_quene.front->mid, 1, (uint8_t *)&status);
                        break;
                    }

                    if ((set_hour > 23) || (set_minute > 60) || (set_sceond > 60))
                    {
                        PR_ERR("Invalid setting time parameter");

                        status = APOLLO_HUB_RESP_ERROR_TIME_INFO;
                        send_resp_msg_to_host(msg_link_quene.front->mid, 1, (uint8_t *)&status);
                        break;
                    }

                    am_hal_rtc_time_get(&hal_time);
                    hal_time.ui32Century = ((set_year / 100) == 20);
                    hal_time.ui32Year = set_year % 100;
                    hal_time.ui32Month = set_month - 1;
                    hal_time.ui32DayOfMonth = set_day;
                    hal_time.ui32Weekday = am_util_time_computeDayofWeek(set_year, set_month, set_day);
                    hal_time.ui32Hour = set_hour;
                    hal_time.ui32Minute = set_minute;
                    hal_time.ui32Second = set_sceond;
                    am_hal_rtc_time_set(&hal_time);

                    send_resp_msg_to_host(msg_link_quene.front->mid, 1, (uint8_t *)&status);

                    break;
                }

                case APOLLO_SENSOR_CONTROL_CMD:
                {
                    uint8_t payload[2] = { 0x00 };

                    PR_INFO("%s, APOLLO_SENSOR_CONTROL_CMD, 0x%02x, 0x%02x", __func__, msg_link_quene.front->data[0], msg_link_quene.front->data[1]);

                    switch (msg_link_quene.front->data[0])
                    {
                        case Accelerometer:
                            if (msg_link_quene.front->data[1])
                            {
                                PR_INFO("will open A sensor");
                                task_list_insert(get_acc_send_msg);
                            }
                            else
                            {
                                PR_INFO("will close A sensor");
                                task_list_remove(get_acc_send_msg);
                            }
                            break;

                        case Gyroscope:
                            if (msg_link_quene.front->data[1])
                            {
                                PR_INFO("will open G sensor");
                                task_list_insert(get_gyro_send_msg);
                            }
                            else
                            {
                                PR_INFO("will close G sensor");
                                task_list_remove(get_gyro_send_msg);
                            }
                            break;

                        case Temperature:
                            if (msg_link_quene.front->data[1])
                            {
                                PR_INFO("will open Temperature sensor");

                                if (!g_bmp280State)
                                {
                                    bmp280_set_power_mode(BMP280_NORMAL_MODE);
                                    task_list_insert(get_bmp280_send_msg);
                                }

                                g_bmp280State |= 0x01;
                            }
                            else
                            {
                                g_bmp280State &= ~(0x01);
                                if (g_bmp280State == 0)
                                {
                                    bmp280_set_config();
                                    task_list_remove(get_bmp280_send_msg);
                                }
                            }
                            break;

                        case Pressure:
                            if (msg_link_quene.front->data[1])
                            {
                                PR_INFO("will open pressure sensor");
                                if (!g_bmp280State)
                                {
                                    bmp280_set_power_mode(BMP280_NORMAL_MODE);
                                    task_list_insert(get_bmp280_send_msg);
                                }

                                g_bmp280State |= 0x02;
                            }
                            else
                            {
                                g_bmp280State &= ~(0x02);
                                if (g_bmp280State == 0)
                                {
                                    bmp280_set_config();
                                    task_list_remove(get_bmp280_send_msg);
                                }
                            }
                            break;

                        case Magnetometer:
                            if (msg_link_quene.front->data[1])
                            {
                                PR_INFO("will open compass sensor");
                                ak099xx_start(10);
                                task_list_insert(get_magnet_send_msg);
                            }
                            else
                            {
                                PR_INFO("will close compass sensor");
                                ak099xx_stop();
                                task_list_remove(get_magnet_send_msg);
                            }
                            break;

                        case HeartRate:
                            if (msg_link_quene.front->data[1])
                            {
                                PR_INFO("will open Heart Rate sensor");
                                pah8series_ppg_dri_HRD_init();
                                pah8series_ppg_HRD_start();
                                task_list_insert(get_irg_hr_send_msg);
                            }
                            else
                            {
                                PR_INFO("will close heart rate sensor");
                                /**if customer want to stop PPG Sensor , please call pah8series_ppg_HRD_stop()
                                * This Function will disable the ppg sensor. */
                                pah8series_ppg_HRD_stop();
                                task_list_remove(get_irg_hr_send_msg);
                            }
                            break;

                        case StepCounter:
                            if (msg_link_quene.front->data[1])
                            {
                                PR_INFO("Will open step detecct");
                                task_list_insert(get_step_send_msg);
                            }
                            else
                            {
                                PR_INFO("Will open close step detect");
                                task_list_remove(get_step_send_msg);
                            }
                            break;

                        case WristTilt:
                            if (msg_link_quene.front->data[1])
                            {
                                PR_INFO("Will open Lift wrist detecct");
                                task_list_insert(get_tilt_status_send_msg);
                            }
                            else
                            {
                                PR_INFO("Will close Lift wrist detecct");
                                task_list_remove(get_tilt_status_send_msg);
                            }
                            break;

                        case OffBodyDetect:
                            if (msg_link_quene.front->data[1])
                            {
                                PR_INFO("Will open take off wrist detecct");
                                pah8series_ppg_dri_HRD_init();
                                pah8series_touch_mode_start();
                                task_list_insert(get_hr_touch_send_msg);
                            }
                            else
                            {
                                PR_INFO("Will close take off wrist detecct");
                                task_list_remove(get_hr_touch_send_msg);
                            }
                            break;

                        default:
                            break;
                    }

                    payload[0] = msg_link_quene.front->data[0];
                    payload[1] = status;
                    send_resp_msg_to_host(msg_link_quene.front->mid, 2, &payload[0]);
                    break;
                }

                case APOLLO_SET_STEPCOUNTER_RESET_CMD:
                    PR_INFO("set APOLLO_SET_STEPCOUNTER_RESET_CMD command");
                    stepcounter_reset();
                    send_resp_msg_to_host(msg_link_quene.front->mid, 1, (uint8_t *)&status);
                    break;

                default:
                    PR_ERR("There is no useful mid: 0x%02x", msg_link_quene.front->mid);
                    break;
            }

            msg_dequene();
        }

        if (*(uint8_t*)&apollo_irq)
        {
            PR_ERR("lsm: %x", apollo_irq);
            apollo_irq.lsm_irq1 = 0;
        }

        if (task_list_num_get())
        {
            call_task_list();
        }

        // Go to Deep Sleep.
        am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);
    }
}
