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
#include "apollo_pah8011.h"

extern volatile msg_link msg_link_quene;
extern uint8_t amotaStart;

extern uint8_t g_UARTRxBuf1[128];
extern uint8_t g_UARTRxBuf1Sta; // 0 0-0x63 0x64
extern uint8_t g_UARTRxBuf2[128];
extern uint8_t g_UARTRxBuf2Sta;

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
    pah8011_init();

    am_hal_interrupt_master_enable();

    // Print the sw infomation.
    END_LINE;
    PR_INFO("Apollo 3 Blue for Sensor Hub, SW ver:%02x.%02x.%02x", 0, 0, 10);

    // init amotas
    dump_ota_status();

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
    }

    while (1)
    {
        // Go to Deep Sleep.
        am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);
    }
}
