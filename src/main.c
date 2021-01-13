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

#include "apollo_lsm6dso.h"
#include "apollo_ak09918.h"
#include "apollo_bmp280.h"
//#include "apollo_pah8011.h"

#include "pah_driver.h"
#include "pah_hrd_function.h"

extern volatile msg_link msg_link_quene;
extern uint8_t amotaStart;

extern uint8_t g_UARTRxBuf[128];
extern uint8_t g_UARTRxBufLen;

static uint8_t g_bmp280State = 0; // 1: temp, 2: pres, 3: all

extern volatile bool        has_interrupt_button;
extern volatile bool        has_interrupt_pah;
extern volatile uint64_t    interrupt_pah_timestamp;
extern float hr;
extern int32_t hr_trust_level;
extern int16_t grade;
extern uint8_t hr_stat;
extern uint8_t touch_stat;

//*****************************************************************************
//
// Related functions that participate in message management
//
//*****************************************************************************
void get_acc_send_msg(void)
{
    float accData[3] = {0};

    if (!lsm6dso_acceleration_get(accData))
    {
        send_event_msg(APOLLO_SENSOR_0_EVNT, (uint8_t*)accData);
        // pr_err("x1=%f,x2=%f,x3=%f\r\n", accData[0], accData[1], accData[2]);

        inform_host();
        wait_fifo_empty();
    }
}

void get_gyro_send_msg(void)
{
    float gyroData[3] = {0};

    if (!lsm6dso_angular_get(gyroData))
    {
        // pr_err("x1=%4.02f,x2=%4.02f,x3=%4.02f\r\n", gyroData[0], gyroData[1], gyroData[2]);
        send_event_msg(APOLLO_SENSOR_1_EVNT, (uint8_t*)gyroData);

        inform_host();
        wait_fifo_empty();
    }
}

void get_bmp280_send_msg(void)
{
    struct bmp280_status m_statue;

    bmp280_get_status(&m_statue);
    if (m_statue.im_update && m_statue.measuring)
    {
        struct bmp280_uncomp_data m_uncomp_data;
        int32_t temp = 0;
        uint32_t press;
        uint8_t ret;

        bmp280_get_uncomp_data(&m_uncomp_data);

        if (g_bmp280State & 0x01) // temperature
        {
            bmp280_get_comp_temp_32bit(&temp, m_uncomp_data.uncomp_temp);
            ret = send_event_msg(APOLLO_SENSOR_2_EVNT, (uint8_t*)&temp);
            if (ret)
            {
                PR_ERR("Temp error");
            }
            else
            {
                inform_host();
                wait_fifo_empty();
            }
        }

        if (g_bmp280State & 0x02) // pressure
        {
            bmp280_get_comp_pres_32bit(&press, m_uncomp_data.uncomp_press);
            ret = send_event_msg(APOLLO_SENSOR_3_EVNT, (uint8_t*)&press);
            if (ret)
            {
                PR_ERR("Temp error");
            }
            else
            {
                inform_host();
                wait_fifo_empty();
            }
        }
    }
}

void get_magnet_send_msg(void)
{
    if (ak099xx_check_rdy())
    {
        int16_t ak_data[3];
        int16_t ak_st[2];
        ak099xx_get_data(ak_data, ak_st);

        if (ak_st[1] &= 0x0004)
        {
            send_event_msg(APOLLO_SENSOR_4_EVNT, (uint8_t*)ak_data);
            // pr_err("x1=%d,x2=%d,x3=%d\r\n", ak_data[0], ak_data[1], ak_data[2]);
            inform_host();
            wait_fifo_empty();
        }
    }
}

void get_irg_hr_send_msg(void)
{
    uint8_t hr_data[10] = {0};
    /**pah8series_ppg_dri_HRD_task, this function only read ppg data & set raw_data format, 
     * Customer can run this function on interrupt_handler or while loop. */
    pah8series_ppg_dri_HRD_task(&has_interrupt_pah ,&has_interrupt_button ,&interrupt_pah_timestamp);

    /**hrd_alg_task, this function will run pixart alg, Customer need run this function on 
     * low priority task before next pah8series_ppg_dri_HRD_task() */
    hrd_alg_task();

    if (hr_stat == MSG_HR_READY)
    {
        hr_stat = 0;
        *(float*)(&hr_data[0]) = hr;
        *(int32_t*)(&hr_data[4]) = hr_trust_level;
        *(int16_t*)(&hr_data[8]) = grade;
        send_event_msg(APOLLO_SENSOR_5_EVNT, (uint8_t*)hr_data);
        inform_host();
    }
}

void get_hr_touch_send_msg(void)
{
    pah8series_touch_mode_dri_task(&has_interrupt_pah ,&has_interrupt_button ,&interrupt_pah_timestamp);

    if (touch_stat & 0x10)
    {
        touch_stat &= 0x0F;
        PR_ERR("Get touch: %d", touch_stat);
        send_event_msg(APOLLO_SENSOR_5_EVNT, &touch_stat);
        inform_host();
    }
}

void get_step_send_msg(void)
{
    uint16_t step;
    static uint16_t last_steps;

    step = lsm6dso_step_get();

    if (last_steps != step)
    {
        PR_INFO("Get step: %d", step);
        send_event_msg(APOLLO_SENSOR_6_EVNT, (uint8_t*)&step);
        inform_host();
        last_steps = step;
    }
}

void get_tilt_status_send_msg(void)
{
    float accData[3] = {0};
    static uint8_t lift_wrist_state = 0;

    if (lsm6dso_tilt_status())
    {
        if (!lsm6dso_acceleration_get(accData))
        {
            if (accData[2] < -860)
            {
                if (lift_wrist_state == 0)
                {
                    lift_wrist_state = 1;
                    PR_ERR("Lift the wrist.");
                    send_event_msg(APOLLO_SENSOR_7_EVNT, (uint8_t*)&lift_wrist_state);
                    inform_host();
                }
            }
            else if (accData[0] < -920 || accData[0] > 920)
            {
                if (lift_wrist_state == 1)
                {
                    lift_wrist_state = 0;
                    PR_ERR("Put down the wrist.");
                    send_event_msg(APOLLO_SENSOR_7_EVNT, (uint8_t*)&lift_wrist_state);
                    inform_host();
                }
            }
        }
    }
}

static uint8_t strtou8(char* nptr)
{
    uint8_t ret;

    if (*nptr < '0' || *nptr > '9')
        return 0;
    
    ret = (*nptr - '0') * 10;
    nptr++;

    if (*nptr < '0' || *nptr > '9')
        return 0;
    
    ret += *nptr - '0';
    return ret;
}

static void send_test_msg(char* nptr)
{
    uart_print("###");
    uart_print(nptr);
    uart_print("***\r\n");
}
//*****************************************************************************
//
// Main
//
//*****************************************************************************
int main(void)
{
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
        // For Factory Test
        if (g_UARTRxBufLen > 0)
        {
            uint32_t i = 0;
            char* reciveCmd = NULL;
            char* reciveCmdTail = NULL;
            uint8_t testCmd;
            char msgBuf[128];
            float lsm6dData[3] = {0};
            int16_t compass_data[3];
            int16_t compass_st[2];

            // Wait for a frame to complete.
            am_util_delay_ms(3);
            reciveCmd = strstr((const char*)g_UARTRxBuf, "###");
            if (reciveCmd != NULL)
            {
                reciveCmdTail = strstr((const char*)g_UARTRxBuf, "***");
                if (reciveCmdTail != NULL)
                {
                    if(reciveCmdTail - reciveCmd == 5)
                    {
                        reciveCmd += 3;
                        *reciveCmdTail = '\0';
                        testCmd = strtou8(reciveCmd);
                        switch (testCmd)
                        {
                            case 1:
                                PR_INFO("MCU status test.");
                                send_test_msg("");
                                break;
                            case 2:
                                PR_INFO("MCU version check.");
                                am_util_stdio_sprintf(msgBuf,"%02X.%02X.%02X", sw_version[0], APOLLO3_HUB_VER1, APOLLO3_HUB_VER2);
                                send_test_msg(msgBuf);
                            case 3:
                                PR_INFO("Gravity sensor test.");
                                for (i = 0; i < 800; i++)
                                {
                                    if (!lsm6dso_acceleration_get(lsm6dData))
                                        break;
                                    am_util_delay_ms(10);
                                }
                                if (i == 800)
                                {
                                    PR_ERR("Gravity sensor test ERROR.");
                                    send_test_msg("Gravity sensor failed to pick up data.");
                                }
                                else
                                {
                                    PR_INFO("Gravity sensor test successed.");
                                    am_util_stdio_sprintf(msgBuf, "Gravity-value: x = %4.2f, y = %4.2f, z = %4.2f",
                                            lsm6dData[0], lsm6dData[1], lsm6dData[2]);
                                    send_test_msg(msgBuf);
                                }
                                break;
                            case 4:
                                PR_INFO("Gyroscope sensor test.");
                                for (i = 0; i < 800; i++)
                                {
                                    if (!lsm6dso_angular_get(lsm6dData))
                                        break;
                                    am_util_delay_ms(10);
                                }
                                if (i == 800)
                                {
                                    PR_ERR("Gyroscope sensor test ERROR.");
                                    send_test_msg("Gyroscope sensor failed to pick up data.");
                                }
                                else
                                {
                                    PR_INFO("Gyroscope sensor test successed.");
                                    am_util_stdio_sprintf(msgBuf, "Gyroscope-value: x = %4.2f, y = %4.2f, z = %4.2f",
                                            lsm6dData[0], lsm6dData[1], lsm6dData[2]);
                                    send_test_msg(msgBuf);
                                }
                                break;
                            case 5:
                                PR_INFO("Compass sensor test.");
                                ak099xx_start(10);
                                for (i = 0; i < 800; i++)
                                {
                                    am_util_delay_ms(10);
                                    ak099xx_get_data(compass_data, compass_st);
                                    if (compass_st[1] &= 0x0004)
                                        break;
                                }
                                if (i == 800)
                                {
                                    PR_ERR("Compass sensor test ERROR.");
                                    send_test_msg("Compass sensor failed to pick up data.");
                                }
                                else
                                {
                                    PR_INFO("Compass sensor test successed.");
                                    am_util_stdio_sprintf(msgBuf, "Compass-value: x = %d, y = %d, z = %d",
                                            compass_data[0], compass_data[1], compass_data[2]);
                                    send_test_msg(msgBuf);
                                }
                                ak099xx_stop();
                                break;
                            case 6:
                                PR_INFO("Temperature sensor test.");
                                bmp280_set_power_mode(BMP280_NORMAL_MODE);
                                for (i = 0; i < 800; i++)
                                {
                                    struct bmp280_status m_statue;
                                    am_util_delay_ms(10);
                                    bmp280_get_status(&m_statue);
                                    if (m_statue.im_update && m_statue.measuring)
                                    {
                                        struct bmp280_uncomp_data m_uncomp_data;
                                        int32_t temp = 0;
                                        uint8_t ret;

                                        bmp280_get_uncomp_data(&m_uncomp_data);
                                        bmp280_get_comp_temp_32bit(&temp, m_uncomp_data.uncomp_temp);
                                        am_util_stdio_sprintf(msgBuf, "Temperature-value: %d", temp);
                                        send_test_msg(msgBuf);
                                        break;
                                    }
                                }
                                if (i == 800)
                                {
                                    PR_ERR("Temperature sensor test ERROR.");
                                    send_test_msg("Temperature sensor failed to pick up data.");
                                }
                                bmp280_set_config();
                                break;
                            case 7:
                                PR_INFO("Pressure sensor test.");
                                bmp280_set_power_mode(BMP280_NORMAL_MODE);
                                for (i = 0; i < 800; i++)
                                {
                                    struct bmp280_status m_statue;
                                    am_util_delay_ms(10);
                                    bmp280_get_status(&m_statue);
                                    if (m_statue.im_update && m_statue.measuring)
                                    {
                                        struct bmp280_uncomp_data m_uncomp_data;
                                        uint32_t press;
                                        uint8_t ret;

                                        bmp280_get_uncomp_data(&m_uncomp_data);
                                        bmp280_get_comp_pres_32bit(&press, m_uncomp_data.uncomp_press);
                                        am_util_stdio_sprintf(msgBuf, "Pressure-value: %d", press);
                                        send_test_msg(msgBuf);
                                        break;
                                    }
                                }
                                if (i == 800)
                                {
                                    PR_ERR("Pressure sensor test ERROR.");
                                    send_test_msg("Pressure sensor failed to pick up data.");
                                }
                                bmp280_set_config();
                                break;
                            case 8:
                                PR_INFO("Heart-Rate sensor test.");
                                pah8series_ppg_dri_HRD_init();
                                break;

                            default:
                                PR_ERR("The test command: %d, does not exist.", testCmd);
                                send_test_msg("The test command does not exist.");
                                break;
                        }
                    }
                    else
                    {
                        PR_ERR("Command length incorrect.");
                    }
                }
                else
                {
                    PR_ERR("Did not receive the correct command tail.");
                }
            }
            else
            {
                PR_ERR("Did not receive the correct command header.");
            }
            g_UARTRxBufLen = 0;
        }

        if (msg_link_quene.front != NULL)
        {
            PR_DBG("Recive msg queue mid: %d", msg_link_quene.front->mid);
            switch(msg_link_quene.front->mid)
            {

                case APOLLO_GET_VERSION_CMD:
                    send_resp_msg(msg_link_quene.front->mid, sw_version);
                    break;

                case APOLLO_FW_UPDATA_CMD:
                    amotas_init();
                    send_resp_msg(msg_link_quene.front->mid, NULL);
                    break;

                case APOLLO_FW_UPDATA_DATA:
                    distribute_pack(msg_link_quene.front->len, msg_link_quene.front->data, 0);
                    send_resp_msg(msg_link_quene.front->mid, NULL);
                    break;

                case APOLLO_FW_DATAEND_CMD:
                    send_resp_msg(msg_link_quene.front->mid, NULL);
                    distribute_pack(0, NULL, 1);
                    break;

                case APOLLO_SET_DATE_CMD:
                {
                    uint16_t* set_year = (uint16_t*)msg_link_quene.front->data;
                    uint8_t* set_month = msg_link_quene.front->data + 2;
                    uint8_t* set_day = msg_link_quene.front->data + 3;

                    PR_INFO("SET date: %04d-%02d-%02d", *set_year, *set_month, *set_day);

                    if (1899 > *set_year || *set_year > 2199 || *set_month > 12 || *set_day > 31)
                    {
                        PR_ERR("Invalid setting date parameter");
                        break;
                    }

                    am_hal_rtc_time_get(&hal_time);
                    hal_time.ui32Century = ((*set_year / 100) == 20);
                    hal_time.ui32Year = *set_year % 100;
                    hal_time.ui32Month = *set_month - 1;
                    hal_time.ui32DayOfMonth = *set_day;
                    hal_time.ui32Weekday = am_util_time_computeDayofWeek(*set_year, *set_month, *set_day);
                    am_hal_rtc_time_set(&hal_time);
                    send_resp_msg(msg_link_quene.front->mid, NULL);
                }
                    break;

                case APOLLO_SET_TIME_CMD:
                {
                    uint8_t set_hour = msg_link_quene.front->data[0];
                    uint8_t set_minute = msg_link_quene.front->data[1];
                    uint8_t set_sceond = msg_link_quene.front->data[2];

                    PR_ERR("SET time: %02d:%02d:%02d", set_hour, set_minute, set_sceond);

                    if (set_hour > 23 || set_minute > 60 || set_sceond > 60)
                    {
                        PR_ERR("Invalid setting time parameter");
                        break;
                    }

                    am_hal_rtc_time_get(&hal_time);
                    hal_time.ui32Hour = set_hour;
                    hal_time.ui32Minute = set_minute;
                    hal_time.ui32Second = set_sceond;
                    am_hal_rtc_time_set(&hal_time);
                }
                    break;

                case APOLLO_ACC_CALI_CMD:
                {
                    uint8_t ret;
                    int8_t acc_cail[3] = {0};
                    ret = lsm6dso_acc_cali();
                    if (ret)
                    {
                        acc_cail[0] = 0xFF;
                        send_resp_msg(msg_link_quene.front->mid, (uint8_t*)acc_cail);
                        break;
                    }

                    lsm6dso_get_acc_cali_data((uint8_t*)acc_cail);
                    send_resp_msg(msg_link_quene.front->mid, (uint8_t*)acc_cail);
                }
                    break;

                case APOLLO_SET_ACC_CALI_CMD:
                    lsm6dso_set_acc_cali_data(msg_link_quene.front->data);
                    send_resp_msg(msg_link_quene.front->mid, NULL);
                    break;

                case APOLLO_SENSOR_0_STOP_CMD:
                    PR_ERR("will close A sensor");

                    task_list_remove(get_acc_send_msg);
                    send_resp_msg(msg_link_quene.front->mid, NULL);
                    break;

                case APOLLO_SENSOR_1_STOP_CMD:
                    PR_ERR("will close G sensor");

                    task_list_remove(get_gyro_send_msg);
                    send_resp_msg(msg_link_quene.front->mid, NULL);
                    break;

                case APOLLO_SENSOR_2_STOP_CMD:
                    PR_ERR("will close Temperature sensor");
                    g_bmp280State &= ~(0x01);
                    if (g_bmp280State == 0)
                    {
                        bmp280_set_config();
                        task_list_remove(get_bmp280_send_msg);
                    }

                    send_resp_msg(msg_link_quene.front->mid, NULL);
                    break;

                case APOLLO_SENSOR_3_STOP_CMD:
                    PR_ERR("will close pressure sensor");
                    g_bmp280State &= ~(0x02);
                    if (g_bmp280State == 0)
                    {
                        bmp280_set_config();
                        task_list_remove(get_bmp280_send_msg);
                    }

                    send_resp_msg(msg_link_quene.front->mid, NULL);
                    break;

                case APOLLO_SENSOR_4_STOP_CMD:
                    PR_ERR("will close compass sensor");
                    ak099xx_stop();
                    task_list_remove(get_magnet_send_msg);
                    send_resp_msg(msg_link_quene.front->mid, NULL);
                    break;

                case APOLLO_SENSOR_5_STOP_CMD:
                    PR_ERR("will close heart rate sensor");
                    /**if customer want to stop PPG Sensor , please call pah8series_ppg_HRD_stop()
                    * This Function will disable the ppg sensor. */
                    pah8series_ppg_HRD_stop();	
                    task_list_remove(get_irg_hr_send_msg);
                    send_resp_msg(msg_link_quene.front->mid, NULL);
                    break;

                case APOLLO_SENSOR_6_STOP_CMD:
                    PR_INFO("Will open close step detect");
                    task_list_remove(get_step_send_msg);
                    send_resp_msg(msg_link_quene.front->mid, NULL);
                    break;

                case APOLLO_SENSOR_7_STOP_CMD:
                    PR_INFO("Will close Lift wrist detecct");
                    task_list_remove(get_tilt_status_send_msg);
                    send_resp_msg(msg_link_quene.front->mid, NULL);
                    break;

                case APOLLO_SENSOR_8_STOP_CMD:
                    PR_INFO("Will close take off wrist detecct");
                    task_list_remove(get_hr_touch_send_msg);
                    send_resp_msg(msg_link_quene.front->mid, NULL);
                    break;

                case APOLLO_SENSOR_0_START_CMD:
                    PR_ERR("will open A sensor");

                    task_list_insert(get_acc_send_msg);
                    send_resp_msg(msg_link_quene.front->mid, NULL);
                    break;

                case APOLLO_SENSOR_1_START_CMD:
                    PR_ERR("will open G sensor");

                    task_list_insert(get_gyro_send_msg);
                    send_resp_msg(msg_link_quene.front->mid, NULL);
                    break;

                case APOLLO_SENSOR_2_START_CMD:  // temperature
                    PR_ERR("will open Temperature sensor");

                    if (!g_bmp280State)
                    {
                        bmp280_set_power_mode(BMP280_NORMAL_MODE);
                        task_list_insert(get_bmp280_send_msg);
                    }

                    g_bmp280State |= 0x01;
                    send_resp_msg(msg_link_quene.front->mid, NULL);
                    break;

                case APOLLO_SENSOR_3_START_CMD:  // pressure
                    PR_ERR("will open pressure sensor");

                    if (!g_bmp280State)
                    {
                        bmp280_set_power_mode(BMP280_NORMAL_MODE);
                        task_list_insert(get_bmp280_send_msg);
                    }

                    g_bmp280State |= 0x02;
                    send_resp_msg(msg_link_quene.front->mid, NULL);
                    break;

                case APOLLO_SENSOR_4_START_CMD:  // compass
                    PR_ERR("will open compass sensor");
                    ak099xx_start(10);
                    task_list_insert(get_magnet_send_msg);
                    send_resp_msg(msg_link_quene.front->mid, NULL);
                    break;

                case APOLLO_SENSOR_5_START_CMD:  // Heart Rate
                    PR_ERR("will open Heart Rate sensor");
                    pah8series_ppg_dri_HRD_init();
                    pah8series_ppg_HRD_start();
                    task_list_insert(get_irg_hr_send_msg);
                    send_resp_msg(msg_link_quene.front->mid, NULL);
                    break;

                case APOLLO_SENSOR_6_START_CMD:  // step detect
                    PR_INFO("Will open step detecct");
                    task_list_insert(get_step_send_msg);
                    send_resp_msg(msg_link_quene.front->mid, NULL);
                    break;

                case APOLLO_SENSOR_7_START_CMD:  // Lift wrist detection
                    PR_INFO("Will open Lift wrist detecct");
                    task_list_insert(get_tilt_status_send_msg);
                    send_resp_msg(msg_link_quene.front->mid, NULL);
                    break;

                case APOLLO_SENSOR_8_START_CMD:  // take off wrist detection
                    PR_INFO("Will open take off wrist detecct");
                    pah8series_ppg_dri_HRD_init();
                    pah8series_touch_mode_start();
                    task_list_insert(get_hr_touch_send_msg);
                    send_resp_msg(msg_link_quene.front->mid, NULL);
                    break;

                default:
                    PR_ERR("There is no useful mid: 0x%02x", msg_link_quene.front->mid);
                    break;

                /*case APOLLO_FW_UPDATA_CMD:
                    amotas_init();
                    send_resp_msg(msg_link_quene.front->mid, NULL);
                    break;*/
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
    }

    while (1)
    {
        // Go to Deep Sleep.
        am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);
    }
}
