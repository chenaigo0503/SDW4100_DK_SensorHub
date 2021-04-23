//*****************************************************************************
//
//! @file apollo_ak09918.c
//!
//! @brief Functions for controlling ak09918(compass sensor)
//!
//! @addtogroup devices External Device Control Library
//! @addtogroup IIC Device Control for ak09918.
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
#include <string.h>
#include "am_bsp.h"
#include "am_mcu_apollo.h"
#include "am_util.h"
#include "apollo_ak09918.h"
#include "apollo_tracelog.h"

void* g_AK09918Hanldle;
static uint32_t akmBuffer[2];

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
void ak099xx_read(uint8_t reg, uint8_t *bufp, uint32_t len)
{
    am_hal_iom_transfer_t       Transaction;
    
    if (bufp == NULL)
        return;

    Transaction.ui32InstrLen    = 1;
    Transaction.ui32Instr       = reg;
    Transaction.eDirection      = AM_HAL_IOM_RX;
    Transaction.ui32NumBytes    = len;
    Transaction.pui32RxBuffer   = akmBuffer;
    Transaction.bContinue       = false;
    Transaction.ui8RepeatCount  = 0;
    Transaction.ui32PauseCondition = 0;
    Transaction.ui32StatusSetClr = 0;
    Transaction.uPeerInfo.ui32I2CDevAddr = AK09918_ADDR;

    am_hal_iom_blocking_transfer(g_AK09918Hanldle, &Transaction);
    memcpy(bufp, akmBuffer, len);
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
static void ak099xx_write(uint8_t reg, uint32_t len, uint8_t *bufp)
{
    am_hal_iom_transfer_t       Transaction;
    
    if (bufp == NULL)
        return;
    
    memcpy(akmBuffer, bufp, len);

    Transaction.ui32InstrLen    = 1;
    Transaction.ui32Instr       = reg;
    Transaction.eDirection      = AM_HAL_IOM_TX;
    Transaction.ui32NumBytes    = len;
    Transaction.pui32TxBuffer   = akmBuffer;
    Transaction.bContinue       = false;
    Transaction.ui8RepeatCount  = 0;
    Transaction.ui32PauseCondition = 0;
    Transaction.ui32StatusSetClr = 0;
    Transaction.uPeerInfo.ui32I2CDevAddr = AK09918_ADDR;

    am_hal_iom_blocking_transfer(g_AK09918Hanldle, &Transaction);
}

void ak099xx_set_mode(const uint8_t mode)
{
    uint8_t i2cData = mode;

    ak099xx_write(AK099XX_REG_CNTL2, 1, &i2cData);
}

void ak099xx_soft_reset(void)
{
    uint8_t i2cData;

    /* Soft Reset */
    i2cData = AK099XX_SOFT_RESET;

    ak099xx_write(AK099XX_REG_CNTL3, 1, &i2cData);

    /* When succeeded, sleep 'Twait' */
    am_util_delay_ms(100);
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
void ak099xx_init(void)
{
    uint16_t ak099ID = 0;
    am_hal_iom_config_t m_sIOMI2cConfig =
    {
        .eInterfaceMode = AM_HAL_IOM_I2C_MODE,
        .ui32ClockFreq = AM_HAL_IOM_400KHZ,
    };

    // init i2c
    if(g_IOMArray[AK09918_IOM_MODULE] == NULL)
    {
        am_hal_iom_initialize(AK09918_IOM_MODULE,
                                &g_IOMArray[AK09918_IOM_MODULE]);
        g_AK09918Hanldle = g_IOMArray[AK09918_IOM_MODULE];
        am_hal_iom_power_ctrl(g_AK09918Hanldle, AM_HAL_SYSCTRL_WAKE, false);

        // Set the required configuration settings for the IOM.
        am_hal_iom_configure(g_AK09918Hanldle, &m_sIOMI2cConfig);

        // Configure the IOM pins.
        am_bsp_iom_pins_enable(AK09918_IOM_MODULE, AM_HAL_IOM_I2C_MODE);
    }
    else
    {
        g_AK09918Hanldle = g_IOMArray[AK09918_IOM_MODULE];
    }

    // Enable the IOM.
    am_hal_iom_enable(g_AK09918Hanldle);

    ak099xx_read(AK099XX_REG_WIA1, (uint8_t*)&ak099ID, 2);

    if(AK09918_WHO_AM_I != ak099ID)
    {
        PR_ERR("ERROR: ak09918 get ID: 0x%04x error.", ak099ID);
    }
    else
    {
        PR_INFO("ak09918 get ID success.");
    }
    
    ak099xx_soft_reset();
}

void ak099xx_start(const int32_t freq)
{
	switch(freq)
	{
		case 10:
			ak099xx_set_mode(AK099XX_MODE_CONT_MEASURE_MODE1);
			break;
		case 20:
            ak099xx_set_mode(AK099XX_MODE_CONT_MEASURE_MODE2);
			break;
		case 50:
			ak099xx_set_mode(AK099XX_MODE_CONT_MEASURE_MODE3);
			break;
		case 100:
			ak099xx_set_mode(AK099XX_MODE_CONT_MEASURE_MODE4);
			break;
		default:
			ak099xx_set_mode(AK099XX_MODE_SNG_MEASURE);
			break;
	}
}

void ak099xx_stop(void)
{
    ak099xx_set_mode(AK099XX_MODE_POWER_DOWN);
}

uint8_t ak099xx_check_rdy(void)
{
    uint8_t i2cData;

    /* Check DRDY bit of ST1 register */
    ak099xx_read(AK099XX_REG_ST1, &i2cData, 1);

    /* AK09911/09912/09913 has only one data.
     * So, return is 0 or 1. */
    return (i2cData & 0x01);
}

void ak099xx_get_data(int16_t data[3], int16_t st[2])
{
    uint8_t i2cData[AK099XX_BDATA_SIZE];
    uint8_t i;

    /* Read data */
    ak099xx_read(AK099XX_REG_ST1, i2cData, AK099XX_BDATA_SIZE);

    for (i = 0; i < 3; i++) {
        /* convert to int16 data */
        data[i] = i2cData[i*2+1] | (i2cData[i*2+2]<<8);
    }

    st[0] = i2cData[0];
    st[1] = i2cData[AK099XX_BDATA_SIZE - 1];
}
