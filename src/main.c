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
#include "apollo_delay2run.h"

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

    am_hal_interrupt_master_enable();

    // Print the sw infomation.
    PR_INFO("\nApollo 3 Blue for Sensor Hub, SW ver:%02x.%02x.%02x\n", 0, 0, 3);

    // init amotas
    dump_ota_status();
    amotas_init();

    while(1)
    {
        static uint32_t read_data = 0;
        static uint16_t data_len;
        if(g_ui32UARTRxIndex == 16712)
        {
            PR_ERR("recive all package\n");
            amotas_cback(1, 0x30, g_UARTRxBlock);
            read_data = 0x30;
            g_ui32UARTRxIndex -= read_data;
            while(g_ui32UARTRxIndex)
            {
                if(g_ui32UARTRxIndex > 0x200)
                    data_len = 0x200;
                else
                    data_len = g_ui32UARTRxIndex;
                amotas_cback(2, data_len, &g_UARTRxBlock[read_data]);
                read_data += data_len;
                g_ui32UARTRxIndex -= data_len;
            }
            
            amotas_cback(3, 0, NULL);
            amotas_cback(4, 0, NULL);
            while(1);
        }
    }

    while (1)
    {
        // Go to Deep Sleep.
        am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);
    }
}
