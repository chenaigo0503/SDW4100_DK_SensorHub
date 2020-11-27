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
#include <string.h>
#include "am_bsp.h"
#include "am_mcu_apollo.h"
#include "am_util.h"
#include "apollo_bmp280.h"
#include "apollo_tracelog.h"

void* g_BMP280Hanldle;
static uint32_t bmp280Buf[3];
static struct bmp280_calib_param calib_param;
static struct bmp280_config conf;
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
void bmp280_read(uint8_t reg, uint8_t *bufp, uint32_t len)
{
    am_hal_iom_transfer_t       Transaction;

    Transaction.ui32InstrLen    = 1;
    Transaction.ui32Instr       = reg;
    Transaction.eDirection      = AM_HAL_IOM_RX;
    Transaction.ui32NumBytes    = len;
    Transaction.pui32RxBuffer   = bmp280Buf;
    Transaction.bContinue       = false;
    Transaction.ui8RepeatCount  = 0;
    Transaction.ui32PauseCondition = 0;
    Transaction.ui32StatusSetClr = 0;
    Transaction.uPeerInfo.ui32I2CDevAddr = BMP280_ADDR;

    PR_ERR("R: %d", am_hal_iom_blocking_transfer(g_BMP280Hanldle, &Transaction));
    memcpy(bufp, bmp280Buf, len);
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
static void bmp280_write(uint8_t reg, uint8_t *bufp, uint32_t len)
{
    am_hal_iom_transfer_t       Transaction;
    
    if (reg == NULL || bufp == NULL)
        return;

    memcpy(bmp280Buf, bufp, len);

    Transaction.ui32InstrLen    = 1;
    Transaction.ui32Instr       = reg;
    Transaction.eDirection      = AM_HAL_IOM_TX;
    Transaction.ui32NumBytes    = len;
    Transaction.pui32TxBuffer   = bmp280Buf;
    Transaction.bContinue       = false;
    Transaction.ui8RepeatCount  = 0;
    Transaction.ui32PauseCondition = 0;
    Transaction.ui32StatusSetClr = 0;
    Transaction.uPeerInfo.ui32I2CDevAddr = BMP280_ADDR;

    PR_ERR("W: %d", am_hal_iom_blocking_transfer(g_BMP280Hanldle, &Transaction));
}

/*!
 * @brief This API reads the data from the given register address of the
 * sensor.
 */
void bmp280_get_regs(uint8_t reg_addr, uint8_t *reg_data, uint8_t len)
{
    bmp280_write(reg_addr, reg_data, len);
}

/*!
 * @brief This internal API interleaves the register addresses and respective
 * register data for a burst write
 */
static void interleave_data(const uint8_t *reg_addr, uint8_t *temp_buff, const uint8_t *reg_data, uint8_t len)
{
    uint8_t index;

    for (index = 1; index < len; index++)
    {
        temp_buff[(index * 2) - 1] = reg_addr[index];
        temp_buff[index * 2] = reg_data[index];
    }
}

/*!
 * @brief This API writes the given data to the register addresses
 * of the sensor.
 */
int8_t bmp280_set_regs(uint8_t *reg_addr, const uint8_t *reg_data, uint8_t len)
{
    uint8_t temp_buff[8]; /* Typically not to write more than 4 registers */
    uint16_t temp_len;

    if (len > 4)
        len = 4;
    
    if (len != 0)
    {
        temp_buff[0] = reg_data[0];
        
        /* Burst write mode */
        if (len > 1)
        {
            /* Interleave register address w.r.t data for burst write*/
            interleave_data(reg_addr, temp_buff, reg_data, len);
            temp_len = ((len * 2) - 1);
        }
        else
        {
            temp_len = len;
        }

        bmp280_write(reg_addr[0], temp_buff, temp_len);
    }
    
}
/*!
 * @brief This API triggers the soft reset of the sensor.
 */
void bmp280_soft_reset(void)
{
    uint8_t reg_addr = BMP280_SOFT_RESET_ADDR;
    uint8_t soft_rst_cmd = BMP280_SOFT_RESET_CMD;

    bmp280_set_regs(&reg_addr, &soft_rst_cmd, 1);

    /* As per the datasheet, startup time is 2 ms. */
    am_util_delay_ms(2);
}

/*!
 * @brief This API is used to read the calibration parameters used
 * for calculating the compensated data.
 */
static void get_calib_param(void)
{
    uint8_t temp[BMP280_CALIB_DATA_SIZE] = { 0 };

    bmp280_read(BMP280_DIG_T1_LSB_ADDR, temp, BMP280_CALIB_DATA_SIZE);
    calib_param.dig_t1 = (uint16_t) (((uint16_t) temp[BMP280_DIG_T1_MSB_POS] << 8) | ((uint16_t) temp[BMP280_DIG_T1_LSB_POS]));
    calib_param.dig_t2 = (int16_t) (((int16_t) temp[BMP280_DIG_T2_MSB_POS] << 8) | ((int16_t) temp[BMP280_DIG_T2_LSB_POS]));
    calib_param.dig_t3 = (int16_t) (((int16_t) temp[BMP280_DIG_T3_MSB_POS] << 8) | ((int16_t) temp[BMP280_DIG_T3_LSB_POS]));
    calib_param.dig_p1 = (uint16_t) (((uint16_t) temp[BMP280_DIG_P1_MSB_POS] << 8) | ((uint16_t) temp[BMP280_DIG_P1_LSB_POS]));
    calib_param.dig_p2 = (int16_t) (((int16_t) temp[BMP280_DIG_P2_MSB_POS] << 8) | ((int16_t) temp[BMP280_DIG_P2_LSB_POS]));
    calib_param.dig_p3 = (int16_t) (((int16_t) temp[BMP280_DIG_P3_MSB_POS] << 8) | ((int16_t) temp[BMP280_DIG_P3_LSB_POS]));
    calib_param.dig_p4 = (int16_t) (((int16_t) temp[BMP280_DIG_P4_MSB_POS] << 8) | ((int16_t) temp[BMP280_DIG_P4_LSB_POS]));
    calib_param.dig_p5 = (int16_t) (((int16_t) temp[BMP280_DIG_P5_MSB_POS] << 8) | ((int16_t) temp[BMP280_DIG_P5_LSB_POS]));
    calib_param.dig_p6 = (int16_t) (((int16_t) temp[BMP280_DIG_P6_MSB_POS] << 8) | ((int16_t) temp[BMP280_DIG_P6_LSB_POS]));
    calib_param.dig_p7 = (int16_t) (((int16_t) temp[BMP280_DIG_P7_MSB_POS] << 8) | ((int16_t) temp[BMP280_DIG_P7_LSB_POS]));
    calib_param.dig_p8 = (int16_t) (((int16_t) temp[BMP280_DIG_P8_MSB_POS] << 8) | ((int16_t) temp[BMP280_DIG_P8_LSB_POS]));
    calib_param.dig_p9 = (int16_t) (((int16_t) temp[BMP280_DIG_P9_MSB_POS] << 8) | ((int16_t) temp[BMP280_DIG_P9_LSB_POS]));

//    PR_ERR("%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d", calib_param.dig_t1, calib_param.dig_t2, calib_param.dig_t3,
//        calib_param.dig_p1, calib_param.dig_p2, calib_param.dig_p3, calib_param.dig_p4, calib_param.dig_p5, calib_param.dig_p6,
//        calib_param.dig_p7, calib_param.dig_p8, calib_param.dig_p9);
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
    uint8_t BMP280Id = 0;
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
    if(BMP280_CHIP_ID3 != BMP280Id)
    {
        PR_ERR("ERROR: BMP280 get ID: 0x%02x error.", (uint8_t)BMP280Id);
        return;
    }
    else
    {
        PR_INFO("BMP280 get ID success.");
    }
    
    bmp280_soft_reset();
    get_calib_param();

    /* Set values to default */
    conf.filter = BMP280_FILTER_OFF;
    conf.os_pres = BMP280_OS_NONE;
    conf.os_temp = BMP280_OS_NONE;
    conf.odr = BMP280_ODR_0_5_MS;
    conf.spi3w_en = BMP280_SPI3_WIRE_DISABLE;
}

/*!
 * @brief This API reads the data from the ctrl_meas register and config
 * register. It gives the currently set temperature and pressure over-sampling
 * configuration, power mode configuration, sleep duration and
 * IIR filter coefficient.
 */
void bmp280_get_config(void)
{
    uint8_t temp[2] = {0};

    bmp280_get_regs(BMP280_CTRL_MEAS_ADDR, temp, 2);

    conf.os_temp = BMP280_GET_BITS(BMP280_OS_TEMP, temp[0]);
    conf.os_pres = BMP280_GET_BITS(BMP280_OS_PRES, temp[0]);
    conf.odr = BMP280_GET_BITS(BMP280_STANDBY_DURN, temp[1]);
    conf.filter = BMP280_GET_BITS(BMP280_FILTER, temp[1]);
    conf.spi3w_en = BMP280_GET_BITS_POS_0(BMP280_SPI3_ENABLE, temp[1]);
}

