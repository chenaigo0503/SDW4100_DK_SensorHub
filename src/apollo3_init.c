//*****************************************************************************
//
//! @file apollo3_init.c
//!
//! @brief A few init functions for use with apollo3
//
//*****************************************************************************

//*****************************************************************************
//
// Copyright (c) 2020, Thundercomm
// All rights reserved.
//
//*****************************************************************************

#include "apollo3_init.h"
#include "am_bsp.h"

//*****************************************************************************
//
// apollo3_init
//
//*****************************************************************************
void apollo3_init(void)
{
    //
    // Set the clock frequency.
    //
    am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_SYSCLK_MAX, 0);

    //
    // Set the default cache configuration
    //
    am_hal_cachectrl_config(&am_hal_cachectrl_defaults);
    am_hal_cachectrl_enable();

    //
    // Configure the board for low power operation.
    //
    am_bsp_low_power_init();	
}
