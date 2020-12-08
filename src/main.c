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

extern uint8_t g_UARTRxBuf1[128];
extern uint8_t g_UARTRxBuf1Sta; // 0 0-0x63 0x64
extern uint8_t g_UARTRxBuf2[128];
extern uint8_t g_UARTRxBuf2Sta;

static uint8_t g_bmp280State = 0; // 1: temp, 2: pres, 3: all

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
        pr_err("x1=%f,x2=%f,x3=%f\r\n", accData[0], accData[1], accData[2]);

        inform_host();
    }
}

void get_gyro_send_msg(void)
{
    float gyroData[3] = {0};
    
    if (!lsm6dso_angular_get(gyroData))
    {
        send_event_msg(APOLLO_SENSOR_1_EVNT, (uint8_t*)gyroData);

        inform_host();
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
        
        bmp280_get_uncomp_data(&m_uncomp_data);

        if (g_bmp280State & 0x01) // temperature
        {
            bmp280_get_comp_temp_32bit(&temp, m_uncomp_data.uncomp_temp);
            send_event_msg(APOLLO_SENSOR_2_EVNT, (uint8_t*)&temp);
            inform_host();
        }

        if (g_bmp280State & 0x02) // pressure
        {
            bmp280_get_comp_pres_32bit(&press, m_uncomp_data.uncomp_press);
            send_event_msg(APOLLO_SENSOR_3_EVNT, (uint8_t*)&press);
            inform_host();
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
        }
    }
}

//*****************************************************************************
//
// Main
//
//*****************************************************************************
int
main(void)
{
    apollo3_init();

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
    //pah8011_init();
    pah_init();

    am_hal_interrupt_master_enable();

    // Print the sw infomation.
    END_LINE;
    PR_INFO("Apollo 3 Blue for Sensor Hub, SW ver:%02x.%02x.%02x", APOLLO3_HUB_VER0,APOLLO3_HUB_VER1, APOLLO3_HUB_VER2);

    // init amotas
    dump_ota_status();
    demo_ppg_polling_HRD();


    while(1)
    {
        
/*********************************************************************
        // OTA testing through UART
        if(g_UARTRxBuf1Sta == 0x64)
        {
            distribute_pack(g_UARTRxBuf1Sta, g_UARTRxBuf1, 0);
            g_UARTRxBuf1Sta = 0;
        }
        if(g_UARTRxBuf2Sta == 0x64)
        {
            distribute_pack(g_UARTRxBuf2Sta, g_UARTRxBuf2, 0);
            g_UARTRxBuf2Sta = 0;
        }
        if(g_ui32UARTRxIndex == 28096)
        {
            // UART recive all update file
            PR_ERR("recive all package:%x %x", g_UARTRxBuf1Sta, g_UARTRxBuf2Sta);
            if(g_UARTRxBuf1Sta)
                distribute_pack(g_UARTRxBuf1Sta, g_UARTRxBuf1, 1);
            if(g_UARTRxBuf2Sta)
                distribute_pack(g_UARTRxBuf2Sta, g_UARTRxBuf2, 1);
            
            while(1);
        }
***********************************************************************************/
        if (msg_link_quene.front != NULL)
        {
            PR_DBG("Recive msg queue mid: %d", msg_link_quene.front->mid);
            switch(msg_link_quene.front->mid)
            {

                case APOLLO_GET_VERSION_CMD:
                    send_resp_msg(msg_link_quene.front->mid);
                    break;

                case APOLLO_FW_UPDATA_CMD:
                    amotas_init();
                    send_resp_msg(msg_link_quene.front->mid);
                    break;

                case APOLLO_FW_UPDATA_DATA:
                    distribute_pack(msg_link_quene.front->len, msg_link_quene.front->data, 0);
                    send_resp_msg(msg_link_quene.front->mid);
                    break;

                case APOLLO_FW_DATAEND_CMD:
                    distribute_pack(0, NULL, 1);
                    send_resp_msg(msg_link_quene.front->mid);
                    break;

                case APOLLO_SENSOR_0_STOP_CMD:
                    PR_ERR("will close A sensor");

                    task_list_remove(get_acc_send_msg);
                    send_resp_msg(msg_link_quene.front->mid);
                    break;

                case APOLLO_SENSOR_1_STOP_CMD:
                    PR_ERR("will close G sensor");

                    task_list_remove(get_gyro_send_msg);
                    send_resp_msg(msg_link_quene.front->mid);
                    break;

                case APOLLO_SENSOR_2_STOP_CMD:
                    PR_ERR("will close Temperature sensor");
                    g_bmp280State &= ~(0x01);
                    if (g_bmp280State == 0)
                    {
                        bmp280_set_config();
                        task_list_remove(get_bmp280_send_msg);
                    }

                    send_resp_msg(msg_link_quene.front->mid);
                    break;

                case APOLLO_SENSOR_3_STOP_CMD:
                    PR_ERR("will close pressure sensor");
                    g_bmp280State &= ~(0x02);
                    if (g_bmp280State == 0)
                    {
                        bmp280_set_config();
                        task_list_remove(get_bmp280_send_msg);
                    }

                    send_resp_msg(msg_link_quene.front->mid);
                    break;

                case APOLLO_SENSOR_4_STOP_CMD:
                    PR_ERR("will close compass sensor");
                    ak099xx_stop();
                    task_list_remove(get_magnet_send_msg);
                    send_resp_msg(msg_link_quene.front->mid);
                    break;

                case APOLLO_SENSOR_0_START_CMD:
                    PR_ERR("will open A sensor");

                    task_list_insert(get_acc_send_msg);
                    send_resp_msg(msg_link_quene.front->mid);
                    break;

                case APOLLO_SENSOR_1_START_CMD:
                    PR_ERR("will open G sensor");

                    task_list_insert(get_gyro_send_msg);
                    send_resp_msg(msg_link_quene.front->mid);
                    break;

                case APOLLO_SENSOR_2_START_CMD:  // temperature
                    PR_ERR("will open Temperature sensor");

                    if (!g_bmp280State)
                    {
                        bmp280_set_power_mode(BMP280_NORMAL_MODE);
                        task_list_insert(get_bmp280_send_msg);
                    }

                    g_bmp280State |= 0x01;
                    send_resp_msg(msg_link_quene.front->mid);
                    break;

                case APOLLO_SENSOR_3_START_CMD:  // pressure
                    PR_ERR("will open pressure sensor");

                    if (!g_bmp280State)
                    {
                        bmp280_set_power_mode(BMP280_NORMAL_MODE);
                        task_list_insert(get_bmp280_send_msg);
                    }

                    g_bmp280State |= 0x02;
                    send_resp_msg(msg_link_quene.front->mid);
                    break;

                case APOLLO_SENSOR_4_START_CMD:  // compass
                    PR_ERR("will open compass sensor");
                    ak099xx_start(10);
                    task_list_insert(get_magnet_send_msg);
                    send_resp_msg(msg_link_quene.front->mid);
                    break;

                default:
                    PR_ERR("There is no useful mid: 0x%02x", msg_link_quene.front->mid);
                    break;

                /*case APOLLO_FW_UPDATA_CMD:
                    amotas_init();
                    send_resp_msg(msg_link_quene.front->mid);
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
