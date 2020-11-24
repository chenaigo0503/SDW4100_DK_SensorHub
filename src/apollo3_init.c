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
// Variables used globally
//
//*****************************************************************************
void* g_IOMArray[6] = {0};
volatile apollo_irq_c_t apollo_irq;

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
    // Enable the floating point module, and configure the core for lazy
    // stacking.
    //
    am_hal_sysctrl_fpu_enable();
    am_hal_sysctrl_fpu_stacking_enable(true);

    //
    // Configure the board for low power operation.
    //
    am_bsp_low_power_init();	
}

//*****************************************************************************
//
// Interrupt handler for the GPIO pins.
//
//*****************************************************************************
void am_gpio_isr(void)
{
    //
    // Read and clear the GPIO interrupt status.
    //
#if defined(AM_PART_APOLLO3P)
    AM_HAL_GPIO_MASKCREATE(GpioIntStatusMask);

    am_hal_gpio_interrupt_status_get(false, pGpioIntStatusMask);
    am_hal_gpio_interrupt_clear(pGpioIntStatusMask);
    am_hal_gpio_interrupt_service(pGpioIntStatusMask);
#elif defined(AM_PART_APOLLO3)
    uint64_t ui64Status;

    am_hal_gpio_interrupt_status_get(false, &ui64Status);
    am_hal_gpio_interrupt_clear(ui64Status);
    am_hal_gpio_interrupt_service(ui64Status);
#else
    #error Unknown device.
#endif
}
