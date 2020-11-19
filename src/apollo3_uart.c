//*****************************************************************************
//
//! @file hello_world_uart.c
//!
//! @brief A simple "Hello World" example using the UART peripheral.
//
//*****************************************************************************

//*****************************************************************************
//
// Copyright (c) 2020, Thundercomm
// All rights reserved.
//
//*****************************************************************************

#include "apollo3_uart.h"
#include <string.h>

//*****************************************************************************
//
// UART handle.
//
//*****************************************************************************
void *phUART;
volatile uint32_t g_ui32UARTRxIndex = 0;

uint8_t g_UARTRxBuf1[128];
uint8_t g_UARTRxBuf1Sta = 0; // 0 0-0x63 0x64
uint8_t g_UARTRxBuf2[128];
uint8_t g_UARTRxBuf2Sta = 0;

#define CHECK_ERRORS(x)                                                       \
    if ((x) != AM_HAL_STATUS_SUCCESS)                                         \
    {                                                                         \
        error_handler(x);                                                     \
    }

volatile uint32_t ui32LastError;

//*****************************************************************************
//
// Catch HAL errors.
//
//*****************************************************************************
void
error_handler(uint32_t ui32ErrorStatus)
{
    ui32LastError = ui32ErrorStatus;

    while (1);
}

//*****************************************************************************
//
// UART buffers.
//
//*****************************************************************************
uint8_t g_pui8TxBuffer[256];
uint8_t g_pui8RxBuffer[32];

void uart_send_data(uint8_t* pcData, uint32_t dataLen)
{
    // there is bug TX buffer not flush
    am_hal_uart_tx_flush(phUART);
    const am_hal_uart_transfer_t sUartWrite =
    {
        .ui32Direction = AM_HAL_UART_WRITE,
        .pui8Data = pcData,
        .ui32NumBytes = dataLen,
        .ui32TimeoutMs = 0,
        .pui32BytesTransferred = NULL,
    };
    CHECK_ERRORS(am_hal_uart_transfer(phUART, &sUartWrite));
}
//*****************************************************************************
//
// UART0 interrupt handler.
//
//*****************************************************************************
void
am_uart_isr(void)
{
    //
    // Service the FIFOs as necessary, and clear the interrupts.
    //
    uint32_t ui32Status, ui32Idle;
    am_hal_uart_interrupt_status_get(phUART, &ui32Status, true);
    am_hal_uart_interrupt_clear(phUART, ui32Status);
    am_hal_uart_interrupt_service(phUART, ui32Status, &ui32Idle);
    
    if (ui32Status & (AM_HAL_UART_INT_RX_TMOUT | AM_HAL_UART_INT_RX))
    {
        uint32_t ui32BytesRead;

        am_hal_uart_transfer_t sRead =
        {
            .ui32Direction = AM_HAL_UART_READ,
            //.pui8Data = (uint8_t*)&g_UARTRxBlock[g_ui32UARTRxIndex],
            .ui32NumBytes = 23,
            .ui32TimeoutMs = 0,
            .pui32BytesTransferred = &ui32BytesRead,
        };

        if (0 < g_UARTRxBuf1Sta && g_UARTRxBuf1Sta < 0x64)
        {
            sRead.pui8Data = (uint8_t*)&g_UARTRxBuf1[g_UARTRxBuf1Sta];
            am_hal_uart_transfer(phUART, &sRead);
            g_UARTRxBuf1Sta += ui32BytesRead;
            if (g_UARTRxBuf1Sta >= 0x64)
            {
                g_UARTRxBuf2Sta = g_UARTRxBuf1Sta - 0x64;
                g_UARTRxBuf1Sta = 0x64;
                memcpy(g_UARTRxBuf2, &g_UARTRxBuf1[0x64], g_UARTRxBuf2Sta);
            }
        }
        else if (0 < g_UARTRxBuf2Sta && g_UARTRxBuf2Sta < 0x64)
        {
            sRead.pui8Data = (uint8_t*)&g_UARTRxBuf2[g_UARTRxBuf2Sta];
            am_hal_uart_transfer(phUART, &sRead);
            g_UARTRxBuf2Sta += ui32BytesRead;
            if (g_UARTRxBuf2Sta >= 0x64)
            {
                g_UARTRxBuf1Sta = g_UARTRxBuf2Sta - 0x64;
                g_UARTRxBuf2Sta = 0x64;
                memcpy(g_UARTRxBuf1, &g_UARTRxBuf2[0x64], g_UARTRxBuf1Sta);
            }
        }
        else if (g_UARTRxBuf1Sta == 0)
        {
            sRead.pui8Data = g_UARTRxBuf1;
            am_hal_uart_transfer(phUART, &sRead);
            g_UARTRxBuf1Sta += ui32BytesRead;
        }
        else if (g_UARTRxBuf2Sta == 0)
        {
            sRead.pui8Data = g_UARTRxBuf2;
            am_hal_uart_transfer(phUART, &sRead);
            g_UARTRxBuf2Sta += ui32BytesRead;
        }

        g_ui32UARTRxIndex += ui32BytesRead;
    }
}

//*****************************************************************************
//
// UART print string
//
//*****************************************************************************
void uart_print(char *pcStr)
{
    uint32_t ui32StrLen = 0;
    uint32_t ui32BytesWritten = 0;

    // there is bug TX buffer not flush
    am_hal_uart_tx_flush(phUART);

    //
    // Measure the length of the string.
    //
    while (pcStr[ui32StrLen] != 0)
    {
        ui32StrLen++;
    }

    //
    // Print the string via the UART.
    //
    const am_hal_uart_transfer_t sUartWrite =
    {
        .ui32Direction = AM_HAL_UART_WRITE,
        .pui8Data = (uint8_t *) pcStr,
        .ui32NumBytes = ui32StrLen,
        .ui32TimeoutMs = 0,
        .pui32BytesTransferred = &ui32BytesWritten,
    };

    CHECK_ERRORS(am_hal_uart_transfer(phUART, &sUartWrite));

    if (ui32BytesWritten != ui32StrLen)
    {
        //
        // Couldn't send the whole string!!
        //
        while(1);
    }
}


//*****************************************************************************
//
// apollo3_uart_init
//
//*****************************************************************************
int
apollo3_uart_init(void)
{
	am_hal_uart_config_t sUartConfig =
	{
		//
		// Standard UART settings: 115200-8-N-1
		//
		.ui32BaudRate = 115200,
		.ui32DataBits = AM_HAL_UART_DATA_BITS_8,
		.ui32Parity = AM_HAL_UART_PARITY_NONE,
		.ui32StopBits = AM_HAL_UART_ONE_STOP_BIT,
		.ui32FlowControl = AM_HAL_UART_FLOW_CTRL_NONE,

		//
		// Set TX and RX FIFOs to interrupt at half-full.
		//
		.ui32FifoLevels = (AM_HAL_UART_TX_FIFO_1_2 |
						   AM_HAL_UART_RX_FIFO_1_2),

		//
		// Buffers
		//
		.pui8TxBuffer = g_pui8TxBuffer,
		.ui32TxBufferSize = sizeof(g_pui8TxBuffer),
		.pui8RxBuffer = g_pui8RxBuffer,
		.ui32RxBufferSize = sizeof(g_pui8RxBuffer),
	};

    //
    // Initialize the printf interface for UART output.
    //
    CHECK_ERRORS(am_hal_uart_initialize(0, &phUART));
    CHECK_ERRORS(am_hal_uart_power_control(phUART, AM_HAL_SYSCTRL_WAKE, false));
    CHECK_ERRORS(am_hal_uart_configure(phUART, &sUartConfig));

    //
    // Enable the UART pins.
    //
    am_hal_gpio_pinconfig(AM_BSP_GPIO_COM_UART_TX, g_AM_BSP_GPIO_COM_UART_TX);
    am_hal_gpio_pinconfig(AM_BSP_GPIO_COM_UART_RX, g_AM_BSP_GPIO_COM_UART_RX);
    
    // open NVIC for UART0
    am_hal_uart_interrupt_clear(phUART, AM_HAL_UART_INT_RX | AM_HAL_UART_INT_RX_TMOUT);
    am_hal_uart_interrupt_enable(phUART, AM_HAL_UART_INT_RX | AM_HAL_UART_INT_RX_TMOUT);
    NVIC_EnableIRQ((IRQn_Type)(UART0_IRQn + AM_BSP_UART_PRINT_INST));
}
