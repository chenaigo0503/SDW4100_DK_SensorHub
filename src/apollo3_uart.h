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
#ifndef APOLLO3_UART_H
#define APOLLO3_UART_H

#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"

//*****************************************************************************
//
// UART handle.
//
//*****************************************************************************
extern void *phUART;
extern volatile uint32_t g_ui32UARTRxIndex;

extern volatile uint32_t ui32LastError;

//*****************************************************************************
//
// Catch HAL errors.
//
//*****************************************************************************
void error_handler(uint32_t ui32ErrorStatus);

//*****************************************************************************
//
// UART buffers.
//
//*****************************************************************************
extern uint8_t g_pui8TxBuffer[256];
extern uint8_t g_pui8RxBuffer[32];



//*****************************************************************************
//
// UART print string
//
//*****************************************************************************
void uart_print(char *pcStr);


//*****************************************************************************
//
// apollo3_init
//
//*****************************************************************************
int apollo3_uart_init(void);

#endif // APOLLO3_UART_H
