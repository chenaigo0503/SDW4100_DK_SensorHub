//*****************************************************************************
//
//! @file apollo_bmp280.c
//!
//! @brief Functions for controlling bmp280(baromater sensor)
//!
//! @addtogroup devices External Device Control Library
//! @addtogroup IIC Device Control for bmp280.
//! @ingroup devices
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
// Copyright (c) 2020, Thundercomm, Inc.
// All rights reserved.
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include "am_bsp.h"
#include "am_mcu_apollo.h"
#include "am_util.h"
#include "apollo_bmp280.h"
#include "apollo_tracelog.h"

void* g_BMP280Hanldle;

/*
 * @brief  Read generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
void bmp280_read(uint8_t reg, uint32_t *bufp,
                             uint32_t len)
{
    am_hal_iom_transfer_t       Transaction;

    Transaction.ui32InstrLen    = 1;
    Transaction.ui32Instr       = reg;
    Transaction.eDirection      = AM_HAL_IOM_RX;
    Transaction.ui32NumBytes    = len;
    Transaction.pui32RxBuffer   = bufp;
    Transaction.bContinue       = false;
    Transaction.ui8RepeatCount  = 0;
    Transaction.ui32PauseCondition = 0;
    Transaction.ui32StatusSetClr = 0;
    Transaction.uPeerInfo.ui32I2CDevAddr = BMP280_ADDR;

    am_hal_iom_blocking_transfer(g_BMP280Hanldle, &Transaction);
}

/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */
static void platform_write(void *handle, uint8_t reg, uint32_t *bufp,
                              uint32_t len)
{
    am_hal_iom_transfer_t       Transaction;

    Transaction.ui32InstrLen    = 1;
    Transaction.ui32Instr       = reg;
    Transaction.eDirection      = AM_HAL_IOM_TX;
    Transaction.ui32NumBytes    = len;
    Transaction.pui32RxBuffer   = bufp;
    Transaction.bContinue       = false;
    Transaction.ui8RepeatCount  = 0;
    Transaction.ui32PauseCondition = 0;
    Transaction.ui32StatusSetClr = 0;
    Transaction.uPeerInfo.ui32I2CDevAddr = BMP280_ADDR;

    am_hal_iom_blocking_transfer(handle, &Transaction);
}

//*****************************************************************************
//
//! @brief Configures the necessary pins for ak09918
//!
//! This function configures a IIC to drive ak09918.
//!
//! @return None.
//
//*****************************************************************************
void bmp280_init(void)
{
    uint32_t BMP280Id = 0;
    am_hal_iom_config_t m_sIOMI2cConfig =
    {
        .eInterfaceMode = AM_HAL_IOM_I2C_MODE,
        .ui32ClockFreq = AM_HAL_IOM_400KHZ,
    };

    // init i2c
    if(g_IOMArray[BMP280_IOM_MODULE] == NULL)
    {
        am_hal_iom_initialize(BMP280_IOM_MODULE,
                                &g_IOMArray[BMP280_IOM_MODULE]);
        g_BMP280Hanldle = g_IOMArray[BMP280_IOM_MODULE];
        am_hal_iom_power_ctrl(g_BMP280Hanldle, AM_HAL_SYSCTRL_WAKE, false);

        // Set the required configuration settings for the IOM.
        am_hal_iom_configure(g_BMP280Hanldle, &m_sIOMI2cConfig);

        // Configure the IOM pins.
        am_bsp_iom_pins_enable(BMP280_IOM_MODULE, AM_HAL_IOM_I2C_MODE);
    }
    else
    {
        g_BMP280Hanldle = g_IOMArray[BMP280_IOM_MODULE];
    }

    // Enable the IOM.
    am_hal_iom_enable(g_BMP280Hanldle);
    
    bmp280_read(BMP280_CHIP_ID_ADDR, &BMP280Id, 1);
    if(BMP280_CHIP_ID3 != (uint8_t)BMP280Id)
    {
        PR_ERR("ERROR: BMP280 get ID: 0x%02x error.\n", (uint8_t)BMP280Id);
    }
    else
    {
        PR_INFO("BMP280 get ID success.\n");
    }
}

#if 0
//*****************************************************************************
//
//! @brief Disables an array of LEDs
//!
//! @param psLEDs is an array of LED structures.
//! @param ui32NumLEDs is the total number of LEDs in the array.
//!
//! This function disables the GPIOs for an array of LEDs.
//!
//! @return None.
//
//*****************************************************************************
void
am_devices_led_array_disable(am_devices_led_t *psLEDs, uint32_t ui32NumLEDs)
{
    if ( (psLEDs == NULL)                       ||
         (ui32NumLEDs > MAX_LEDS) )
    {
        return;
    }

    //
    // Loop through the list of LEDs, configuring each one individually.
    //
    for ( uint32_t i = 0; i < ui32NumLEDs; i++ )
    {
        if ( psLEDs[i].ui32GPIONumber >= AM_HAL_GPIO_MAX_PADS )
        {
            continue;
        }

#if defined(AM_PART_APOLLO4) || defined(AM_PART_APOLLO4B)
        am_hal_gpio_pinconfig((psLEDs + i)->ui32GPIONumber, am_hal_gpio_pincfg_disabled);
#else
#if AM_APOLLO3_GPIO
        am_hal_gpio_pinconfig((psLEDs + i)->ui32GPIONumber, g_AM_HAL_GPIO_DISABLE);
#else // AM_APOLLO3_GPIO
        am_hal_gpio_pin_config((psLEDs + i)->ui32GPIONumber, AM_HAL_GPIO_DISABLE);
#endif // AM_APOLLO3_GPIO
#endif
    }
}

//*****************************************************************************
//
//! @brief Configures the necessary pins for an array of LEDs
//!
//! @param psLEDs is an array of LED structures.
//! @param ui32NumLEDs is the total number of LEDs in the array.
//!
//! This function configures the GPIOs for an array of LEDs.
//!
//! @return None.
//
//*****************************************************************************
void
am_devices_led_array_init(am_devices_led_t *psLEDs, uint32_t ui32NumLEDs)
{
    uint32_t i;

    if ( (psLEDs == NULL)                       ||
         (ui32NumLEDs > MAX_LEDS) )
    {
        return;
    }

    //
    // Loop through the list of LEDs, configuring each one individually.
    //
    for ( i = 0; i < ui32NumLEDs; i++ )
    {
        am_devices_led_init(psLEDs + i);
    }
}

//*****************************************************************************
//
//! @brief Turns on the requested LED.
//!
//! @param psLEDs is an array of LED structures.
//! @param ui32LEDNum is the LED number for the light to turn on.
//!
//! This function turns on a single LED.
//!
//! @return None.
//
//*****************************************************************************
void
am_devices_led_on(am_devices_led_t *psLEDs, uint32_t ui32LEDNum)
{
    if ( (psLEDs == NULL)                       ||
         (ui32LEDNum >= MAX_LEDS)               ||
         (psLEDs[ui32LEDNum].ui32GPIONumber >= AM_HAL_GPIO_MAX_PADS) )
    {
        return;
    }

#if defined(AM_PART_APOLLO4) || defined(AM_PART_APOLLO4B)
    //
    // Handle Direct Drive Versus 3-State (with pull-up or no buffer).
    //
    if ( AM_DEVICES_LED_POL_DIRECT_DRIVE_M & psLEDs[ui32LEDNum].ui32Polarity )
    {
        //
        // Set the output to the correct state for the LED.
        //
        am_hal_gpio_state_write(psLEDs[ui32LEDNum].ui32GPIONumber,
                                psLEDs[ui32LEDNum].ui32Polarity & AM_DEVICES_LED_POL_POLARITY_M ?
                                AM_HAL_GPIO_OUTPUT_SET : AM_HAL_GPIO_OUTPUT_CLEAR);
    }
    else
    {
        //
        // Turn on the output driver for the LED.
        //
        am_hal_gpio_state_write(psLEDs[ui32LEDNum].ui32GPIONumber,
                                AM_HAL_GPIO_OUTPUT_TRISTATE_ENABLE);
    }
#else
#if (1 == AM_APOLLO3_GPIO)
    //
    // Handle Direct Drive Versus 3-State (with pull-up or no buffer).
    //
    if ( AM_DEVICES_LED_POL_DIRECT_DRIVE_M & psLEDs[ui32LEDNum].ui32Polarity )
    {
        //
        // Set the output to the correct state for the LED.
        //
        am_hal_gpio_state_write(psLEDs[ui32LEDNum].ui32GPIONumber,
                                psLEDs[ui32LEDNum].ui32Polarity & AM_DEVICES_LED_POL_POLARITY_M ?
                                AM_HAL_GPIO_OUTPUT_SET : AM_HAL_GPIO_OUTPUT_CLEAR);
    }
    else
    {
        //
        // Turn on the output driver for the LED.
        //
        am_hal_gpio_state_write(psLEDs[ui32LEDNum].ui32GPIONumber,
                                AM_HAL_GPIO_OUTPUT_TRISTATE_ENABLE);
    }
#else // AM_APOLLO3_GPIO
    //
    // Handle Direct Drive Versus 3-State (with pull-up or no buffer).
    //
    if ( AM_DEVICES_LED_POL_DIRECT_DRIVE_M & psLEDs[ui32LEDNum].ui32Polarity )
    {
        //
        // Set the output to the correct state for the LED.
        //
        am_hal_gpio_out_bit_replace(psLEDs[ui32LEDNum].ui32GPIONumber,
                                    psLEDs[ui32LEDNum].ui32Polarity &
                                    AM_DEVICES_LED_POL_POLARITY_M );
    }
    else
    {
        //
        // Turn on the output driver for the LED.
        //
        am_hal_gpio_out_enable_bit_set(psLEDs[ui32LEDNum].ui32GPIONumber);
    }
#endif // AM_APOLLO3_GPIO
#endif
}

//*****************************************************************************
//
//! @brief Turns off the requested LED.
//!
//! @param psLEDs is an array of LED structures.
//! @param ui32LEDNum is the LED number for the light to turn off.
//!
//! This function turns off a single LED.
//!
//! @return None.
//
//*****************************************************************************
void
am_devices_led_off(am_devices_led_t *psLEDs, uint32_t ui32LEDNum)
{
    if ( (psLEDs == NULL)                       ||
         (ui32LEDNum >= MAX_LEDS)               ||
         (psLEDs[ui32LEDNum].ui32GPIONumber >= AM_HAL_GPIO_MAX_PADS) )
    {
        return;
    }

#if defined(AM_PART_APOLLO4) || defined(AM_PART_APOLLO4B)
    //
    // Handle Direct Drive Versus 3-State (with pull-up or no buffer).
    //
    if ( AM_DEVICES_LED_POL_DIRECT_DRIVE_M & psLEDs[ui32LEDNum].ui32Polarity )
    {
        //
        // Set the output to the correct state for the LED.
        //
        am_hal_gpio_state_write(psLEDs[ui32LEDNum].ui32GPIONumber,
                                psLEDs[ui32LEDNum].ui32Polarity & AM_DEVICES_LED_POL_POLARITY_M ?
                                AM_HAL_GPIO_OUTPUT_CLEAR : AM_HAL_GPIO_OUTPUT_SET);
    }
    else
    {
        //
        // Turn off the output driver for the LED.
        //
        am_hal_gpio_state_write(psLEDs[ui32LEDNum].ui32GPIONumber,
                                AM_HAL_GPIO_OUTPUT_TRISTATE_DISABLE);
    }
#else
#if (1 == AM_APOLLO3_GPIO)
    //
    // Handle Direct Drive Versus 3-State (with pull-up or no buffer).
    //
    if ( AM_DEVICES_LED_POL_DIRECT_DRIVE_M & psLEDs[ui32LEDNum].ui32Polarity )
    {
        //
        // Set the output to the correct state for the LED.
        //
        am_hal_gpio_state_write(psLEDs[ui32LEDNum].ui32GPIONumber,
                                psLEDs[ui32LEDNum].ui32Polarity & AM_DEVICES_LED_POL_POLARITY_M ?
                                AM_HAL_GPIO_OUTPUT_CLEAR : AM_HAL_GPIO_OUTPUT_SET);
    }
    else
    {
        //
        // Turn off the output driver for the LED.
        //
        am_hal_gpio_state_write(psLEDs[ui32LEDNum].ui32GPIONumber,
                                AM_HAL_GPIO_OUTPUT_TRISTATE_DISABLE);
    }
#else // AM_APOLLO3_GPIO
    //
    // Handle Direct Drive Versus 3-State (with pull-up or no buffer).
    //
    if ( AM_DEVICES_LED_POL_DIRECT_DRIVE_M & psLEDs[ui32LEDNum].ui32Polarity )
    {
        //
        // Set the output to the correct state for the LED.
        //
        am_hal_gpio_out_bit_replace(psLEDs[ui32LEDNum].ui32GPIONumber,
                                    !(psLEDs[ui32LEDNum].ui32Polarity &
                                      AM_DEVICES_LED_POL_POLARITY_M) );
    }
    else
    {
        //
        // Turn off the output driver for the LED.
        //
        am_hal_gpio_out_enable_bit_clear(psLEDs[ui32LEDNum].ui32GPIONumber);
    }
#endif // AM_APOLLO3_GPIO
#endif
}

//*****************************************************************************
//
//! @brief Toggles the requested LED.
//!
//! @param psLEDs is an array of LED structures.
//! @param ui32LEDNum is the LED number for the light to toggle.
//!
//! This function toggles a single LED.
//!
//! @return None.
//
//*****************************************************************************
void
am_devices_led_toggle(am_devices_led_t *psLEDs, uint32_t ui32LEDNum)
{
    if ( (psLEDs == NULL)                       ||
         (ui32LEDNum >= MAX_LEDS)               ||
         (psLEDs[ui32LEDNum].ui32GPIONumber >= AM_HAL_GPIO_MAX_PADS) )
    {
        return;
    }

#if defined(AM_PART_APOLLO4) || defined(AM_PART_APOLLO4B)
    //
    // Handle Direct Drive Versus 3-State (with pull-up or no buffer).
    //
    if ( AM_DEVICES_LED_POL_DIRECT_DRIVE_M & psLEDs[ui32LEDNum].ui32Polarity )
    {
        am_hal_gpio_state_write(psLEDs[ui32LEDNum].ui32GPIONumber,
                                AM_HAL_GPIO_OUTPUT_TOGGLE);
    }
    else
    {
        uint32_t ui32Ret, ui32Value;

        //
        // Check to see if the LED pin is enabled.
        //
        ui32Ret = am_hal_gpio_state_read(psLEDs[ui32LEDNum].ui32GPIONumber,
                                         AM_HAL_GPIO_ENABLE_READ, &ui32Value);

        if ( ui32Ret == AM_HAL_STATUS_SUCCESS )
        {
            if ( ui32Value )
            {
                //
                // If it was enabled, turn if off.
                //
                am_hal_gpio_state_write(psLEDs[ui32LEDNum].ui32GPIONumber,
                                        AM_HAL_GPIO_OUTPUT_TRISTATE_DISABLE);
            }
            else
            {
                //
                // If it was not enabled, turn it on.
                //
                am_hal_gpio_state_write(psLEDs[ui32LEDNum].ui32GPIONumber,
                                        AM_HAL_GPIO_OUTPUT_TRISTATE_ENABLE);
            }
        }
    }
#else
#if (1 == AM_APOLLO3_GPIO)
    //
    // Handle Direct Drive Versus 3-State (with pull-up or no buffer).
    //
    if ( AM_DEVICES_LED_POL_DIRECT_DRIVE_M & psLEDs[ui32LEDNum].ui32Polarity )
    {
        am_hal_gpio_state_write(psLEDs[ui32LEDNum].ui32GPIONumber,
                                AM_HAL_GPIO_OUTPUT_TOGGLE);
    }
    else
    {
        uint32_t ui32Ret, ui32Value;

        //
        // Check to see if the LED pin is enabled.
        //
        ui32Ret = am_hal_gpio_state_read(psLEDs[ui32LEDNum].ui32GPIONumber,
                                         AM_HAL_GPIO_ENABLE_READ, &ui32Value);

        if ( ui32Ret == AM_HAL_STATUS_SUCCESS )
        {
            if ( ui32Value )
            {
                //
                // If it was enabled, turn if off.
                //
                am_hal_gpio_state_write(psLEDs[ui32LEDNum].ui32GPIONumber,
                                        AM_HAL_GPIO_OUTPUT_TRISTATE_DISABLE);
            }
            else
            {
                //
                // If it was not enabled, turn it on.
                //
                am_hal_gpio_state_write(psLEDs[ui32LEDNum].ui32GPIONumber,
                                        AM_HAL_GPIO_OUTPUT_TRISTATE_ENABLE);
            }
        }
    }
#else // AM_APOLLO3_GPIO
    //
    // Handle Direct Drive Versus 3-State (with pull-up or no buffer).
    //
    if ( AM_DEVICES_LED_POL_DIRECT_DRIVE_M & psLEDs[ui32LEDNum].ui32Polarity )
    {
        am_hal_gpio_out_bit_toggle(psLEDs[ui32LEDNum].ui32GPIONumber);
    }
    else
    {
        //
        // Check to see if the LED pin is enabled.
        //
        if ( am_hal_gpio_out_enable_bit_get(psLEDs[ui32LEDNum].ui32GPIONumber) )
        {
            //
            // If it was enabled, turn if off.
            //
            am_hal_gpio_out_enable_bit_clear(psLEDs[ui32LEDNum].ui32GPIONumber);
        }
        else
        {
            //
            // If it was not enabled, turn if on.
            //
            am_hal_gpio_out_enable_bit_set(psLEDs[ui32LEDNum].ui32GPIONumber);
        }
    }
#endif // AM_APOLLO3_GPIO
#endif
}

//*****************************************************************************
//
//! @brief Gets the state of the requested LED.
//!
//! @param psLEDs is an array of LED structures.
//! @param ui32LEDNum is the LED to check.
//!
//! This function checks the state of a single LED.
//!
//! @return true if the LED is on.
//
//*****************************************************************************
bool
am_devices_led_get(am_devices_led_t *psLEDs, uint32_t ui32LEDNum)
{
    if ( (psLEDs == NULL)                       ||
         (ui32LEDNum >= MAX_LEDS)               ||
         (psLEDs[ui32LEDNum].ui32GPIONumber >= AM_HAL_GPIO_MAX_PADS) )
    {
        return false;   // No error return, so return as off
    }

#if defined(AM_PART_APOLLO4) || defined(AM_PART_APOLLO4B)
    uint32_t ui32Ret, ui32Value;
    am_hal_gpio_read_type_e eReadType;

    eReadType = AM_DEVICES_LED_POL_DIRECT_DRIVE_M & psLEDs[ui32LEDNum].ui32Polarity ?
                AM_HAL_GPIO_OUTPUT_READ : AM_HAL_GPIO_ENABLE_READ;

    ui32Ret = am_hal_gpio_state_read(psLEDs[ui32LEDNum].ui32GPIONumber,
                                     eReadType, &ui32Value);

    if ( ui32Ret == AM_HAL_STATUS_SUCCESS )
    {
        return (bool)ui32Value;
    }
    else
    {
        return false;
    }
#else
#if (1 == AM_APOLLO3_GPIO)
    uint32_t ui32Ret, ui32Value;
    am_hal_gpio_read_type_e eReadType;

    eReadType = AM_DEVICES_LED_POL_DIRECT_DRIVE_M & psLEDs[ui32LEDNum].ui32Polarity ?
                AM_HAL_GPIO_OUTPUT_READ : AM_HAL_GPIO_ENABLE_READ;

    ui32Ret = am_hal_gpio_state_read(psLEDs[ui32LEDNum].ui32GPIONumber,
                                     eReadType, &ui32Value);

    if ( ui32Ret == AM_HAL_STATUS_SUCCESS )
    {
        return (bool)ui32Value;
    }
    else
    {
        return false;
    }
#else // AM_APOLLO3_GPIO
    //
    // Handle Direct Drive Versus 3-State (with pull-up or no buffer).
    //
    if ( AM_DEVICES_LED_POL_DIRECT_DRIVE_M & psLEDs[ui32LEDNum].ui32Polarity )
    {
        //
        // Mask to the GPIO bit position for this GPIO number.
        //
        uint64_t ui64Mask = ((uint64_t)0x01l) << psLEDs[ui32LEDNum].ui32GPIONumber;

        //
        // Extract the state of this bit and return it.
        //
        return !!(am_hal_gpio_out_read() & ui64Mask);
    }
    else
    {
        return am_hal_gpio_out_enable_bit_get(psLEDs[ui32LEDNum].ui32GPIONumber);
    }
#endif // AM_APOLLO3_GPIO
#endif
}

//*****************************************************************************
//
//! @brief Display a binary value using LEDs.
//!
//! @param psLEDs is an array of LED structures.
//! @param ui32NumLEDs is the number of LEDs in the array.
//! @param ui32Value is the value to display on the LEDs.
//!
//! This function displays a value in binary across an array of LEDs.
//!
//! @return true if the LED is on.
//
//*****************************************************************************
void
am_devices_led_array_out(am_devices_led_t *psLEDs, uint32_t ui32NumLEDs,
                         uint32_t ui32Value)
{
    uint32_t i;

    for ( i = 0; i < ui32NumLEDs; i++ )
    {
        if ( ui32Value & (1 << i) )
        {
            am_devices_led_on(psLEDs, i);
        }
        else
        {
            am_devices_led_off(psLEDs, i);
        }
    }
}
//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
#endif