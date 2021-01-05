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

#include "apollo3_ios.h"
#include "apollo3_init.h"
#include "apollo_tracelog.h"
#include "apollo_delay2run.h"
#include "apollo_message.h"

//*****************************************************************************
//
// IOS handle.
//
//*****************************************************************************
void *g_pIOSHandle;
uint8_t g_IOSFifoBuffer[APOLLO3_IOS_TXBUF_MAX];

static void pulldown_iosint(void* inPara)
{
    am_hal_gpio_state_write(APOLLO3_IOSINT_PIN, AM_HAL_GPIO_OUTPUT_CLEAR);
}

//*****************************************************************************
//
// Configure the IO slave.
//
//*****************************************************************************
void ios_init(void)
{
#ifdef APOLLO3_IOS_USE_SPI
    // SPI Slave Configuration
    am_hal_ios_config_t m_IOSSpiConfig =
    {
        // Configure the IOS in SPI mode.
        .ui32InterfaceSelect = AM_HAL_IOS_USE_SPI,

        // Eliminate the "read-only" section, so an external host can use the
        // entire "direct write" section.
        .ui32ROBase = 0x78,

        // Making the "FIFO" section as big as possible.
        .ui32FIFOBase = 0x80,

        // We don't need any RAM space, so extend the FIFO all the way to the end
        // of the LRAM.
        .ui32RAMBase = 0x100,

        // FIFO Threshold - set to half the size
        .ui32FIFOThreshold = 0x20,

        .pui8SRAMBuffer = g_IOSFifoBuffer,
        .ui32SRAMBufferCap = APOLLO3_IOS_TXBUF_MAX,
    };

    // Configure SPI interface
    am_bsp_ios_pins_enable(0, AM_HAL_IOS_USE_SPI);
    //
    // Configure the IOS interface and LRAM structure.
    //
    am_hal_ios_initialize(0, &g_pIOSHandle);
    am_hal_ios_power_ctrl(g_pIOSHandle, AM_HAL_SYSCTRL_WAKE, false);
    am_hal_ios_configure(g_pIOSHandle, &m_IOSSpiConfig);
#endif
#ifdef APOLLO3_IOS_USE_I2C
    // I2C Slave Configuration
    am_hal_ios_config_t m_IOSI2cConfig =
    {
        // Configure the IOS in I2C mode.
        .ui32InterfaceSelect = AM_HAL_IOS_USE_I2C | AM_HAL_IOS_I2C_ADDRESS(I2C_ADDR << 1),

        // Eliminate the "read-only" section, so an external host can use the
        // entire "direct write" section.
        .ui32ROBase = 0x78,

        // Set the FIFO base to the maximum value, making the "direct write"
        // section as big as possible.
        .ui32FIFOBase = 0x80,

        // We don't need any RAM space, so extend the FIFO all the way to the end
        // of the LRAM.
        .ui32RAMBase = 0x100,

        // FIFO Threshold - set to half the size
        .ui32FIFOThreshold = 0x40,

        .pui8SRAMBuffer = g_IOSFifoBuffer,
        .ui32SRAMBufferCap = APOLLO3_IOS_TXBUF_MAX,
    };

    // Configure I2C interface
    am_bsp_ios_pins_enable(0, AM_HAL_IOS_USE_I2C);
    //
    // Configure the IOS interface and LRAM structure.
    //
    am_hal_ios_initialize(0, &g_pIOSHandle);
    am_hal_ios_power_ctrl(g_pIOSHandle, AM_HAL_SYSCTRL_WAKE, false);
    am_hal_ios_configure(g_pIOSHandle, &m_IOSI2cConfig);
#endif
    // Clear out any IOS register-access interrupts that may be active, and
    // enable interrupts for the registers we're interested in.
    //
    am_hal_ios_interrupt_clear(g_pIOSHandle, AM_HAL_IOS_INT_ALL);
    am_hal_ios_interrupt_enable(g_pIOSHandle, AM_HAL_IOS_INT_ERR | AM_HAL_IOS_INT_FSIZE);
    am_hal_ios_interrupt_enable(g_pIOSHandle, AM_HAL_IOS_XCMP_INT);
    
    // Set the bit in the NVIC to accept access interrupts from the IO Slave.
    NVIC_EnableIRQ(IOSLAVE_IRQn);

    // Set up the IOSINT interrupt pin
    if (sw_version[0] == 0)
    {
        // Use timer to control notification pin
        delay2run_init();
        am_hal_gpio_state_write(APOLLO3_IOSINT_PIN, AM_HAL_GPIO_OUTPUT_CLEAR);
        am_hal_gpio_pinconfig(APOLLO3_IOSINT_PIN, g_AM_HAL_GPIO_OUTPUT);
    }
    else
    {
        am_hal_gpio_pincfg_t m_ios_int_pin =
        {
            .uFuncSel            = AM_HAL_PIN_4_SLINT,
            .eDriveStrength      = AM_HAL_GPIO_PIN_DRIVESTRENGTH_2MA,
            .eCEpol              = AM_HAL_GPIO_PIN_CEPOL_ACTIVEHIGH,
            .eGPOutcfg           = AM_HAL_GPIO_PIN_OUTCFG_OPENDRAIN,
        };

        // Set up the IOSINT interrupt pin
        am_hal_gpio_pinconfig(4, m_ios_int_pin);
    }
}

// Inform host of new data available to read
void inform_host(void)
{
    uint32_t ui32Arg = AM_IOSTEST_IOSTOHOST_DATAAVAIL_INTMASK;
    uint32_t gpio_state;

    // Update FIFOCTR for host to read
    am_hal_ios_control(g_pIOSHandle, AM_HAL_IOS_REQ_FIFO_UPDATE_CTR, NULL);
    // Interrupt the host
    if (sw_version[0] == 0)
    {
        // Just in case the GPIO hasn't pulled down yet
        am_hal_gpio_state_read(APOLLO3_IOSINT_PIN, AM_HAL_GPIO_OUTPUT_READ, &gpio_state);
        if (gpio_state == 1)
            am_hal_gpio_state_write(APOLLO3_IOSINT_PIN, AM_HAL_GPIO_OUTPUT_CLEAR);

        delay_to_run(50, pulldown_iosint, NULL);
        am_hal_gpio_state_write(APOLLO3_IOSINT_PIN, AM_HAL_GPIO_OUTPUT_SET);
    }
    am_hal_ios_control(g_pIOSHandle, AM_HAL_IOS_REQ_HOST_INTSET, &ui32Arg);
}

void wait_fifo_empty(void)
{
    uint32_t iosFifoSpace;

    do
    {
        am_util_delay_ms(5);
        am_hal_ios_fifo_space_used(g_pIOSHandle, &iosFifoSpace);
    }
    while (iosFifoSpace);
}

//*****************************************************************************
//
// IO Slave Main ISR.
//
//*****************************************************************************
void am_ioslave_ios_isr(void)
{
    uint32_t ui32Status;
    int ret;

    // Check to see what caused this interrupt, then clear the bit from the
    // interrupt register.
    am_hal_ios_interrupt_status_get(g_pIOSHandle, false, &ui32Status);
    am_hal_ios_interrupt_clear(g_pIOSHandle, ui32Status);

    if (ui32Status & AM_HAL_IOS_INT_FUNDFL)
    {
        PR_ERR("Hitting underflow for the requested IOS FIFO transfer");
        // We should never hit this case unless the threshold has beeen set
        // incorrect, or we are unable to handle the data rate
        // ERROR!
        am_hal_debug_assert_msg(0,
            "Hitting underflow for the requested IOS FIFO transfer.");
    }

    if (ui32Status & AM_HAL_IOS_INT_ERR)
    {
        // We should never hit this case
        // ERROR!
        am_hal_debug_assert_msg(0, "Hitting ERROR case.");
    }

    if (ui32Status & AM_HAL_IOS_INT_FSIZE)
    {
        // Service the I2C slave FIFO if necessary.
        am_hal_ios_interrupt_service(g_pIOSHandle, ui32Status);
    }

    if (ui32Status & AM_HAL_IOS_INT_XCMPWR)
    {
        // Set up a pointer for writing 32-bit aligned packets through
        // the IO slave interface.
        ret = unpack_data((uint8_t*)am_hal_ios_pui8LRAM);
        PR_DBG("upack data: ret = %d", ret);
    }
}
