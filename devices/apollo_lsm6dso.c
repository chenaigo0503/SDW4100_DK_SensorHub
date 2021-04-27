//*****************************************************************************
//
//! @file apollo_lsm6dso.c
//!
//! @brief Functions for controlling lsm6dso(A+G sensor)
//!
//! @addtogroup devices External Device Control Library
//! @addtogroup SPI Device Control for lsm6dso.
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
#include <math.h>
#include "am_bsp.h"
#include "am_mcu_apollo.h"
#include "am_util.h"
#include "apollo_lsm6dso.h"
#include "apollo_tracelog.h"

stmdev_ctx_t g_Lsm6dsoCtx;
static uint32_t lsmBuffer[2];

typedef union{
  int16_t i16bit[3];
  uint8_t u8bit[6];
} axis3bit16_t;
static axis3bit16_t data_raw_acceleration;
static axis3bit16_t data_raw_angular_rate;
/**
  * @defgroup  LSM6DSOX_Private_functions
  * @brief     Section collect all the utility functions needed by APIs.
  * @{
  *
  */

static void bytecpy(uint8_t *target, uint8_t *source)
{
  if ( (target != NULL) && (source != NULL) ) {
    *target = *source;
  }
}

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
static void lsm6d_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint32_t len)
{
    am_hal_iom_transfer_t       Transaction;

    // Temporary fix for hardware bugs.
    memset(&Transaction, 0, sizeof(Transaction));

    Transaction.ui32InstrLen    = 1;
    Transaction.ui32Instr       = reg | 0x80;
    Transaction.eDirection      = AM_HAL_IOM_RX;
    Transaction.ui32NumBytes    = len;
    Transaction.pui32RxBuffer   = lsmBuffer;
    Transaction.bContinue       = false;
    Transaction.ui8RepeatCount  = 0;
    Transaction.ui32PauseCondition = 0;
    Transaction.ui32StatusSetClr = 0;

    if (sw_version[0] == 0)
        am_hal_gpio_state_write(LSM6DSO_PIN_CE, AM_HAL_GPIO_OUTPUT_CLEAR);
    else
        Transaction.uPeerInfo.ui32SpiChipSelect = AM_BSP_IOM0_CS_CHNL;

    am_hal_iom_blocking_transfer(handle, &Transaction);
    memcpy(bufp, lsmBuffer, len);
    if (sw_version[0] == 0)
        am_hal_gpio_state_write(LSM6DSO_PIN_CE, AM_HAL_GPIO_OUTPUT_SET);
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
static void lsm6d_write(void *handle, uint8_t reg, uint8_t *bufp,
                              uint32_t len)
{
    am_hal_iom_transfer_t       Transaction;

    memcpy(lsmBuffer, bufp, len);
    memset(&Transaction, 0, sizeof(Transaction));

    Transaction.ui32InstrLen    = 1;
    Transaction.ui32Instr       = reg;
    Transaction.eDirection      = AM_HAL_IOM_TX;
    Transaction.ui32NumBytes    = len;
    Transaction.pui32TxBuffer   = lsmBuffer;
    Transaction.bContinue       = false;
    Transaction.ui8RepeatCount  = 0;
    Transaction.ui32PauseCondition = 0;
    Transaction.ui32StatusSetClr = 0;

    if (sw_version[0] == 0)
        am_hal_gpio_state_write(LSM6DSO_PIN_CE, AM_HAL_GPIO_OUTPUT_CLEAR);
    else
        Transaction.uPeerInfo.ui32SpiChipSelect = AM_BSP_IOM0_CS_CHNL;

    am_hal_iom_blocking_transfer(handle, &Transaction);
    if (sw_version[0] == 0)
        am_hal_gpio_state_write(LSM6DSO_PIN_CE, AM_HAL_GPIO_OUTPUT_SET);
}

/**
  * @defgroup  LSM6DSO_Sensitivity
  * @brief     These functions convert raw-data into engineering units.
  * @{
  *
*/
float_t lsm6dso_from_fs2_to_mg(int16_t lsb)
{
  return ((float_t)lsb) * 0.061f;
}

float_t lsm6dso_from_fs4_to_mg(int16_t lsb)
{
  return ((float_t)lsb) * 0.122f;
}

float_t lsm6dso_from_fs8_to_mg(int16_t lsb)
{
  return ((float_t)lsb) * 0.244f;
}

float_t lsm6dso_from_fs16_to_mg(int16_t lsb)
{
  return ((float_t)lsb) *0.488f;
}

float_t lsm6dso_from_fs125_to_mdps(int16_t lsb)
{
  return ((float_t)lsb) *4.375f;
}

float_t lsm6dso_from_fs500_to_mdps(int16_t lsb)
{
  return ((float_t)lsb) *17.50f;
}

float_t lsm6dso_from_fs250_to_mdps(int16_t lsb)
{
  return ((float_t)lsb) *8.750f;
}

float_t lsm6dso_from_fs1000_to_mdps(int16_t lsb)
{
  return ((float_t)lsb) *35.0f;
}

float_t lsm6dso_from_fs2000_to_mdps(int16_t lsb)
{
  return ((float_t)lsb) *70.0f;
}

float_t lsm6dso_from_lsb_to_celsius(int16_t lsb)
{
  return (((float_t)lsb / 256.0f) + 25.0f);
}

float_t lsm6dso_from_lsb_to_nsec(int16_t lsb)
{
  return ((float_t)lsb * 25000.0f);
}

/**
  * @brief  Device "Who am I".[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  buff     buffer that stores data read
  *
  */
static void lsm6dso_device_id_get(stmdev_ctx_t *ctx, uint8_t *buff)
{
    ctx->read_reg(ctx->handle, LSM6DSO_OFFSET_ID, buff, 1);
}

/**
  * @brief  Software reset. Restore the default values
  *         in user registers[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of sw_reset in reg CTRL3_C
  *
  */
static void lsm6dso_reset_set(stmdev_ctx_t *ctx, uint8_t val)
{
    lsm6dso_ctrl3_c_t reg;

    ctx->read_reg(ctx->handle, LSM6DSO_CTRL3_C, (uint8_t*)&reg, 1);
    reg.sw_reset = val;
    ctx->write_reg(ctx->handle, LSM6DSO_CTRL3_C, (uint8_t*)&reg, 1);
}

/**
  * @brief  Software reset. Restore the default values in user registers.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of sw_reset in reg CTRL3_C
  *
  */
static void lsm6dso_reset_get(stmdev_ctx_t *ctx, uint8_t *val)
{
    lsm6dso_ctrl3_c_t reg;

    ctx->read_reg(ctx->handle, LSM6DSO_CTRL3_C, (uint8_t*)&reg, 1);
    *val = reg.sw_reset;
}

/**
  * @brief  I3C Enable/Disable communication protocol[.set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of i3c_disable
  *                                    in reg CTRL9_XL
  *
  */
static void lsm6dso_i3c_disable_set(stmdev_ctx_t *ctx, lsm6dso_i3c_disable_t val)
{
    lsm6dso_i3c_bus_avb_t i3c_bus_avb;
    lsm6dso_ctrl9_xl_t ctrl9_xl;

    ctx->read_reg(ctx->handle, LSM6DSO_CTRL9_XL, (uint8_t*)&ctrl9_xl, 1);
    ctrl9_xl.i3c_disable = ((uint8_t)val & 0x80U) >> 7;

    // spec
    ctrl9_xl.not_used_01 = 1;

    ctx->write_reg(ctx->handle, LSM6DSO_CTRL9_XL, (uint8_t*)&ctrl9_xl, 1);
    ctx->read_reg(ctx->handle, LSM6DSO_I3C_BUS_AVB, (uint8_t*)&i3c_bus_avb, 1);
    i3c_bus_avb.i3c_bus_avb_sel = (uint8_t)val & 0x03U;
    ctx->write_reg(ctx->handle, LSM6DSO_I3C_BUS_AVB, (uint8_t*)&i3c_bus_avb, 1);
}

/**
  * @brief  Accelerometer UI data rate selection.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of odr_xl in reg CTRL1_XL
  *
  */
static void lsm6dso_xl_data_rate_set(stmdev_ctx_t *ctx, lsm6dso_odr_xl_t val)
{
    lsm6dso_odr_xl_t odr_xl =  val;
    lsm6dso_emb_fsm_enable_t fsm_enable;
    lsm6dso_fsm_odr_t fsm_odr;
    lsm6dso_ctrl1_xl_t reg;

  /* Check the Finite State Machine data rate constraints */
    lsm6dso_fsm_enable_get(ctx, &fsm_enable);
    if ( (fsm_enable.fsm_enable_a.fsm1_en  |
          fsm_enable.fsm_enable_a.fsm2_en  |
          fsm_enable.fsm_enable_a.fsm3_en  |
          fsm_enable.fsm_enable_a.fsm4_en  |
          fsm_enable.fsm_enable_a.fsm5_en  |
          fsm_enable.fsm_enable_a.fsm6_en  |
          fsm_enable.fsm_enable_a.fsm7_en  |
          fsm_enable.fsm_enable_a.fsm8_en  |
          fsm_enable.fsm_enable_b.fsm9_en  |
          fsm_enable.fsm_enable_b.fsm10_en |
          fsm_enable.fsm_enable_b.fsm11_en |
          fsm_enable.fsm_enable_b.fsm12_en |
          fsm_enable.fsm_enable_b.fsm13_en |
          fsm_enable.fsm_enable_b.fsm14_en |
          fsm_enable.fsm_enable_b.fsm15_en |
          fsm_enable.fsm_enable_b.fsm16_en ) == PROPERTY_ENABLE ){
        lsm6dso_fsm_data_rate_get(ctx, &fsm_odr);
        switch (fsm_odr)
        {
            case LSM6DSO_ODR_FSM_12Hz5:
                if (val == LSM6DSO_XL_ODR_OFF)
                {
                    odr_xl = LSM6DSO_XL_ODR_12Hz5;
                }
                else
                {
                    odr_xl = val;
                }
                break;

            case LSM6DSO_ODR_FSM_26Hz:
                if (val == LSM6DSO_XL_ODR_OFF)
                {
                    odr_xl = LSM6DSO_XL_ODR_26Hz;
                }
                else if (val == LSM6DSO_XL_ODR_12Hz5)
                {
                    odr_xl = LSM6DSO_XL_ODR_26Hz;
                }
                else
                {
                    odr_xl = val;
                }
                break;

            case LSM6DSO_ODR_FSM_52Hz:
                if (val == LSM6DSO_XL_ODR_OFF)
                {
                    odr_xl = LSM6DSO_XL_ODR_52Hz;
                }
                else if (val == LSM6DSO_XL_ODR_12Hz5)
                {
                    odr_xl = LSM6DSO_XL_ODR_52Hz;
                }
                else if (val == LSM6DSO_XL_ODR_26Hz)
                {
                    odr_xl = LSM6DSO_XL_ODR_52Hz;
                }
                else
                {
                    odr_xl = val;
                }
                break;

            case LSM6DSO_ODR_FSM_104Hz:
                if (val == LSM6DSO_XL_ODR_OFF)
                {
                    odr_xl = LSM6DSO_XL_ODR_104Hz;
                }
                else if (val == LSM6DSO_XL_ODR_12Hz5)
                {
                    odr_xl = LSM6DSO_XL_ODR_104Hz;
                }
                else if (val == LSM6DSO_XL_ODR_26Hz)
                {
                    odr_xl = LSM6DSO_XL_ODR_104Hz;
                }
                else if (val == LSM6DSO_XL_ODR_52Hz)
                {
                    odr_xl = LSM6DSO_XL_ODR_104Hz;
                }
                else
                {
                    odr_xl = val;
                }
                break;

            default:
                odr_xl = val;
                break;
        }
    }
    ctx->read_reg(ctx->handle, LSM6DSO_CTRL1_XL, (uint8_t*)&reg, 1);
    reg.odr_xl = (uint8_t)odr_xl;
    ctx->write_reg(ctx->handle, LSM6DSO_CTRL1_XL, (uint8_t*)&reg, 1);
}

/**
  * @brief  Gyroscope UI data rate selection.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of odr_g in reg CTRL2_G
  *
  */
void lsm6dso_gy_data_rate_set(stmdev_ctx_t *ctx, lsm6dso_odr_g_t val)
{
    lsm6dso_odr_g_t odr_gy =  val;
    lsm6dso_emb_fsm_enable_t fsm_enable;
    lsm6dso_fsm_odr_t fsm_odr;
    lsm6dso_ctrl2_g_t reg;

    /* Check the Finite State Machine data rate constraints */
    lsm6dso_fsm_enable_get(ctx, &fsm_enable);
    if ( (fsm_enable.fsm_enable_a.fsm1_en  |
          fsm_enable.fsm_enable_a.fsm2_en  |
          fsm_enable.fsm_enable_a.fsm3_en  |
          fsm_enable.fsm_enable_a.fsm4_en  |
          fsm_enable.fsm_enable_a.fsm5_en  |
          fsm_enable.fsm_enable_a.fsm6_en  |
          fsm_enable.fsm_enable_a.fsm7_en  |
          fsm_enable.fsm_enable_a.fsm8_en  |
          fsm_enable.fsm_enable_b.fsm9_en  |
          fsm_enable.fsm_enable_b.fsm10_en |
          fsm_enable.fsm_enable_b.fsm11_en |
          fsm_enable.fsm_enable_b.fsm12_en |
          fsm_enable.fsm_enable_b.fsm13_en |
          fsm_enable.fsm_enable_b.fsm14_en |
          fsm_enable.fsm_enable_b.fsm15_en |
          fsm_enable.fsm_enable_b.fsm16_en ) == PROPERTY_ENABLE )
    {
        lsm6dso_fsm_data_rate_get(ctx, &fsm_odr);
        switch (fsm_odr)
        {
            case LSM6DSO_ODR_FSM_12Hz5:
                if (val == LSM6DSO_GY_ODR_OFF)
                    odr_gy = LSM6DSO_GY_ODR_12Hz5;
                else
                    odr_gy = val;
                break;

            case LSM6DSO_ODR_FSM_26Hz:
                if (val == LSM6DSO_GY_ODR_OFF)
                    odr_gy = LSM6DSO_GY_ODR_26Hz;
                else if (val == LSM6DSO_GY_ODR_12Hz5)
                    odr_gy = LSM6DSO_GY_ODR_26Hz;
                else
                    odr_gy = val;
                break;

            case LSM6DSO_ODR_FSM_52Hz:
                if (val == LSM6DSO_GY_ODR_OFF)
                    odr_gy = LSM6DSO_GY_ODR_52Hz;
                else if (val == LSM6DSO_GY_ODR_12Hz5)
                    odr_gy = LSM6DSO_GY_ODR_52Hz;
                else if (val == LSM6DSO_GY_ODR_26Hz)
                    odr_gy = LSM6DSO_GY_ODR_52Hz;
                else
                    odr_gy = val;
                break;

            case LSM6DSO_ODR_FSM_104Hz:
                if (val == LSM6DSO_GY_ODR_OFF)
                    odr_gy = LSM6DSO_GY_ODR_104Hz;
                else if (val == LSM6DSO_GY_ODR_12Hz5)
                    odr_gy = LSM6DSO_GY_ODR_104Hz;
                else if (val == LSM6DSO_GY_ODR_26Hz)
                    odr_gy = LSM6DSO_GY_ODR_104Hz;
                else if (val == LSM6DSO_GY_ODR_52Hz)
                    odr_gy = LSM6DSO_GY_ODR_104Hz;
                else
                    odr_gy = val;
                break;

            default:
                odr_gy = val;
                break;
        }
    }

        ctx->read_reg(ctx->handle, LSM6DSO_CTRL2_G, (uint8_t*)&reg, 1);
        reg.odr_g = (uint8_t) odr_gy;
        ctx->write_reg(ctx->handle, LSM6DSO_CTRL2_G, (uint8_t*)&reg, 1);
}

/**
  * @brief  Block data update.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of bdu in reg CTRL3_C
  *
  */
void lsm6dso_block_data_update_set(stmdev_ctx_t *ctx, uint8_t val)
{
    lsm6dso_ctrl3_c_t reg;

    ctx->read_reg(ctx->handle, LSM6DSO_CTRL3_C, (uint8_t*)&reg, 1);
    reg.bdu = val;
    ctx->write_reg(ctx->handle, LSM6DSO_CTRL3_C, (uint8_t*)&reg, 1);
}

/**
  * @defgroup  LSM6DSO_Data_Generation
  * @brief     This section groups all the functions concerning
  *            data generation.
  *
*/

/**
  * @brief  Accelerometer full-scale selection.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of fs_xl in reg CTRL1_XL
  *
  */
void lsm6dso_xl_full_scale_set(stmdev_ctx_t *ctx,
                                  lsm6dso_fs_xl_t val)
{
    lsm6dso_ctrl1_xl_t reg;

    ctx->read_reg(ctx->handle, LSM6DSO_CTRL1_XL, (uint8_t*)&reg, 1);
    reg.fs_xl = (uint8_t)val;
    ctx->write_reg(ctx->handle, LSM6DSO_CTRL1_XL, (uint8_t*)&reg, 1);
}

/**
  * @brief  Gyroscope UI chain full-scale selection.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of fs_g in reg CTRL2_G
  *
  */
void lsm6dso_gy_full_scale_set(stmdev_ctx_t *ctx, lsm6dso_fs_g_t val)
{
    lsm6dso_ctrl2_g_t reg;

    ctx->read_reg(ctx->handle, LSM6DSO_CTRL2_G, (uint8_t*)&reg, 1);
    reg.fs_g = (uint8_t) val;
    ctx->write_reg(ctx->handle, LSM6DSO_CTRL2_G, (uint8_t*)&reg, 1);
}

/**
  * @brief  Wake up duration event.[set]
  *         1LSb = 1 / ODR
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of wake_dur in reg WAKE_UP_DUR
  *
  */
void lsm6dso_wkup_dur_set(stmdev_ctx_t *ctx, uint8_t val)
{
    lsm6dso_wake_up_dur_t reg;

    ctx->read_reg(ctx->handle, LSM6DSO_WAKE_UP_DUR, (uint8_t*)&reg, 1);
    reg.wake_dur = val;
    ctx->write_reg(ctx->handle, LSM6DSO_WAKE_UP_DUR, (uint8_t*)&reg, 1);
}

/**
  * @brief  Duration to go in sleep mode.[set]
  *         1 LSb = 512 / ODR
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of sleep_dur in reg WAKE_UP_DUR
  *
  */
void lsm6dso_act_sleep_dur_set(stmdev_ctx_t *ctx, uint8_t val)
{
    lsm6dso_wake_up_dur_t reg;

    ctx->read_reg(ctx->handle, LSM6DSO_WAKE_UP_DUR, (uint8_t*)&reg, 1);
    reg.sleep_dur = val;
    ctx->write_reg(ctx->handle, LSM6DSO_WAKE_UP_DUR, (uint8_t*)&reg, 1);
}

/**
  * @brief  Threshold for wakeup: 1 LSB weight depends on WAKE_THS_W in
  *         WAKE_UP_DUR.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of wk_ths in reg WAKE_UP_THS
  *
  */
void lsm6dso_wkup_threshold_set(stmdev_ctx_t *ctx, uint8_t val)
{
    lsm6dso_wake_up_ths_t reg;

    ctx->read_reg(ctx->handle, LSM6DSO_WAKE_UP_THS, (uint8_t*)&reg, 1);
    reg.wk_ths = val;
    ctx->write_reg(ctx->handle, LSM6DSO_WAKE_UP_THS, (uint8_t*)&reg, 1);
}

/**
  * @brief  Enable inactivity function.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of inact_en in reg TAP_CFG2
  *
  */
void lsm6dso_act_mode_set(stmdev_ctx_t *ctx, lsm6dso_inact_en_t val)
{
    lsm6dso_tap_cfg2_t reg;

    ctx->read_reg(ctx->handle, LSM6DSO_TAP_CFG2, (uint8_t*)&reg, 1);
    reg.inact_en = (uint8_t)val;
    ctx->write_reg(ctx->handle, LSM6DSO_TAP_CFG2, (uint8_t*)&reg, 1);
}

/**
  * @brief  Route interrupt signals on int1 pin.[get]
  *
  * @param  ctx          communication interface handler.(ptr)
  * @param  val          the signals that are routed on int1 pin.(ptr)
  *
  */
void lsm6dso_pin_int1_route_get(stmdev_ctx_t *ctx,
                                    lsm6dso_pin_int1_route_t *val)
{
    lsm6dso_emb_func_int1_t   emb_func_int1;
    lsm6dso_fsm_int1_a_t      fsm_int1_a;
    lsm6dso_fsm_int1_b_t      fsm_int1_b;
    lsm6dso_int1_ctrl_t       int1_ctrl;
    lsm6dso_int2_ctrl_t       int2_ctrl;
    lsm6dso_md2_cfg_t         md2_cfg;
    lsm6dso_md1_cfg_t         md1_cfg;
    lsm6dso_ctrl4_c_t         ctrl4_c;

    lsm6dso_mem_bank_set(ctx, LSM6DSO_EMBEDDED_FUNC_BANK);
    ctx->read_reg(ctx->handle, LSM6DSO_EMB_FUNC_INT1, (uint8_t*)&emb_func_int1, 1);
    ctx->read_reg(ctx->handle, LSM6DSO_FSM_INT1_A, (uint8_t*)&fsm_int1_a, 1);
    ctx->read_reg(ctx->handle, LSM6DSO_FSM_INT1_B, (uint8_t*)&fsm_int1_b, 1);
    lsm6dso_mem_bank_set(ctx, LSM6DSO_USER_BANK);
    ctx->read_reg(ctx->handle, LSM6DSO_INT1_CTRL, (uint8_t*)&int1_ctrl, 1);
    ctx->read_reg(ctx->handle, LSM6DSO_MD1_CFG, (uint8_t*)&md1_cfg, 1);
    ctx->read_reg(ctx->handle, LSM6DSO_CTRL4_C, (uint8_t*)&ctrl4_c, 1);
    if (ctrl4_c.int2_on_int1 == PROPERTY_ENABLE)
    {
        ctx->read_reg(ctx->handle, LSM6DSO_INT2_CTRL, (uint8_t*)&int2_ctrl, 1);
        val->drdy_temp = int2_ctrl.int2_drdy_temp;
        ctx->read_reg(ctx->handle, LSM6DSO_MD2_CFG, (uint8_t*)&md2_cfg, 1);
        val->timestamp = md2_cfg.int2_timestamp;
    }
    else
    {
        val->drdy_temp = PROPERTY_DISABLE;
        val->timestamp = PROPERTY_DISABLE;
    }

    val->drdy_xl   = int1_ctrl.int1_drdy_xl;
    val->drdy_g    = int1_ctrl.int1_drdy_g;
    val->boot      = int1_ctrl.int1_boot;
    val->fifo_th   = int1_ctrl.int1_fifo_th;
    val->fifo_ovr  = int1_ctrl.int1_fifo_ovr;
    val->fifo_full = int1_ctrl.int1_fifo_full;
    val->fifo_bdr  = int1_ctrl.int1_cnt_bdr;
    val->den_flag  = int1_ctrl.den_drdy_flag;

    val->sh_endop     = md1_cfg.int1_shub;
    val->six_d        = md1_cfg.int1_6d;
    val->double_tap   = md1_cfg.int1_double_tap;
    val->free_fall    = md1_cfg.int1_ff;
    val->wake_up      = md1_cfg.int1_wu;
    val->single_tap   = md1_cfg.int1_single_tap;
    val->sleep_change = md1_cfg.int1_sleep_change;

    val->step_detector = emb_func_int1.int1_step_detector;
    val->tilt          = emb_func_int1.int1_tilt;
    val->sig_mot       = emb_func_int1.int1_sig_mot;
    val->fsm_lc        = emb_func_int1.int1_fsm_lc;

    val->fsm1 = fsm_int1_a.int1_fsm1;
    val->fsm2 = fsm_int1_a.int1_fsm2;
    val->fsm3 = fsm_int1_a.int1_fsm3;
    val->fsm4 = fsm_int1_a.int1_fsm4;
    val->fsm5 = fsm_int1_a.int1_fsm5;
    val->fsm6 = fsm_int1_a.int1_fsm6;
    val->fsm7 = fsm_int1_a.int1_fsm7;
    val->fsm8 = fsm_int1_a.int1_fsm8;
    val->fsm9  = fsm_int1_b.int1_fsm9;
    val->fsm10 = fsm_int1_b.int1_fsm10;
    val->fsm11 = fsm_int1_b.int1_fsm11;
    val->fsm12 = fsm_int1_b.int1_fsm12;
    val->fsm13 = fsm_int1_b.int1_fsm13;
    val->fsm14 = fsm_int1_b.int1_fsm14;
    val->fsm15 = fsm_int1_b.int1_fsm15;
    val->fsm16 = fsm_int1_b.int1_fsm16;
}

/**
  * @brief  Route interrupt signals on int2 pin.[get]
  *
  * @param  ctx          communication interface handler. Use NULL to ingnore
  *                      this interface.(ptr)
  * @param  aux_ctx      auxiliary communication interface handler. Use NULL
  *                      to ingnore this interface.(ptr)
  * @param  val          the signals that are routed on int2 pin.(ptr)
  *
  */
void lsm6dso_pin_int2_route_get(stmdev_ctx_t *ctx, stmdev_ctx_t *aux_ctx,
                                    lsm6dso_pin_int2_route_t *val)
{
    lsm6dso_emb_func_int2_t  emb_func_int2;
    lsm6dso_fsm_int2_a_t     fsm_int2_a;
    lsm6dso_fsm_int2_b_t     fsm_int2_b;
    lsm6dso_int2_ctrl_t      int2_ctrl;
    lsm6dso_md2_cfg_t        md2_cfg;
    lsm6dso_ctrl4_c_t        ctrl4_c;
    lsm6dso_int_ois_t        int_ois;

    if( aux_ctx != NULL )
    {
        aux_ctx->read_reg(aux_ctx->handle, LSM6DSO_INT_OIS, (uint8_t*)&int_ois, 1);
        val->drdy_ois = int_ois.int2_drdy_ois;
    }

    if( ctx != NULL )
    {
        lsm6dso_mem_bank_set(ctx, LSM6DSO_EMBEDDED_FUNC_BANK);
        ctx->read_reg(ctx->handle, LSM6DSO_EMB_FUNC_INT2, (uint8_t*)&emb_func_int2, 1);
        ctx->read_reg(ctx->handle, LSM6DSO_FSM_INT2_A, (uint8_t*)&fsm_int2_a, 1);
        ctx->read_reg(ctx->handle, LSM6DSO_FSM_INT2_B, (uint8_t*)&fsm_int2_b, 1);

        lsm6dso_mem_bank_set(ctx, LSM6DSO_USER_BANK);
        ctx->read_reg(ctx->handle, LSM6DSO_INT2_CTRL, (uint8_t*)&int2_ctrl, 1);
        ctx->read_reg(ctx->handle, LSM6DSO_MD2_CFG, (uint8_t*)&md2_cfg, 1);

        ctx->read_reg(ctx->handle, LSM6DSO_CTRL4_C, (uint8_t*)&ctrl4_c, 1);
        if (ctrl4_c.int2_on_int1 == PROPERTY_DISABLE)
        {
            ctx->read_reg(ctx->handle, LSM6DSO_INT2_CTRL, (uint8_t*)&int2_ctrl, 1);
            val->drdy_temp = int2_ctrl.int2_drdy_temp;
            ctx->read_reg(ctx->handle, LSM6DSO_MD2_CFG, (uint8_t*)&md2_cfg, 1);
            val->timestamp = md2_cfg.int2_timestamp;
        }
        else
        {
            val->drdy_temp = PROPERTY_DISABLE;
            val->timestamp = PROPERTY_DISABLE;
        }

        val->drdy_xl   = int2_ctrl.int2_drdy_xl;
        val->drdy_g    = int2_ctrl.int2_drdy_g;
        val->drdy_temp = int2_ctrl.int2_drdy_temp;
        val->fifo_th   = int2_ctrl.int2_fifo_th;
        val->fifo_ovr  = int2_ctrl.int2_fifo_ovr;
        val->fifo_full = int2_ctrl.int2_fifo_full;
        val->fifo_bdr   = int2_ctrl.int2_cnt_bdr;

        val->timestamp    = md2_cfg.int2_timestamp;
        val->six_d        = md2_cfg.int2_6d;
        val->double_tap   = md2_cfg.int2_double_tap;
        val->free_fall    = md2_cfg.int2_ff;
        val->wake_up      = md2_cfg.int2_wu;
        val->single_tap   = md2_cfg.int2_single_tap;
        val->sleep_change = md2_cfg.int2_sleep_change;

        val->step_detector = emb_func_int2. int2_step_detector;
        val->tilt          = emb_func_int2.int2_tilt;
        val->fsm_lc        = emb_func_int2.int2_fsm_lc;

        val->fsm1 = fsm_int2_a.int2_fsm1;
        val->fsm2 = fsm_int2_a.int2_fsm2;
        val->fsm3 = fsm_int2_a.int2_fsm3;
        val->fsm4 = fsm_int2_a.int2_fsm4;
        val->fsm5 = fsm_int2_a.int2_fsm5;
        val->fsm6 = fsm_int2_a.int2_fsm6;
        val->fsm7 = fsm_int2_a.int2_fsm7;
        val->fsm8 = fsm_int2_a.int2_fsm8;

        val->fsm9  = fsm_int2_b.int2_fsm9;
        val->fsm10 = fsm_int2_b.int2_fsm10;
        val->fsm11 = fsm_int2_b.int2_fsm11;
        val->fsm12 = fsm_int2_b.int2_fsm12;
        val->fsm13 = fsm_int2_b.int2_fsm13;
        val->fsm14 = fsm_int2_b.int2_fsm14;
        val->fsm15 = fsm_int2_b.int2_fsm15;
        val->fsm16 = fsm_int2_b.int2_fsm16;
    }
}

/**
  * @brief  Route interrupt signals on int1 pin.[set]
  *
  * @param  ctx          communication interface handler.(ptr)
  * @param  val          the signals to route on int1 pin.
  *
  */
void lsm6dso_pin_int1_route_set(stmdev_ctx_t *ctx,
                                    lsm6dso_pin_int1_route_t val)
{
    lsm6dso_pin_int2_route_t  pin_int2_route;
    lsm6dso_emb_func_int1_t   emb_func_int1;
    lsm6dso_fsm_int1_a_t      fsm_int1_a;
    lsm6dso_fsm_int1_b_t      fsm_int1_b;
    lsm6dso_int1_ctrl_t       int1_ctrl;
    lsm6dso_int2_ctrl_t       int2_ctrl;
    lsm6dso_tap_cfg2_t        tap_cfg2;
    lsm6dso_md2_cfg_t         md2_cfg;
    lsm6dso_md1_cfg_t         md1_cfg;
    lsm6dso_ctrl4_c_t         ctrl4_c;

    int1_ctrl.int1_drdy_xl   = val.drdy_xl;
    int1_ctrl.int1_drdy_g    = val.drdy_g;
    int1_ctrl.int1_boot      = val.boot;
    int1_ctrl.int1_fifo_th   = val.fifo_th;
    int1_ctrl.int1_fifo_ovr  = val.fifo_ovr;
    int1_ctrl.int1_fifo_full = val.fifo_full;
    int1_ctrl.int1_cnt_bdr   = val.fifo_bdr;
    int1_ctrl.den_drdy_flag  = val.den_flag;

    md1_cfg.int1_shub         = val.sh_endop;
    md1_cfg.int1_6d           = val.six_d;
    md1_cfg.int1_double_tap   = val.double_tap;
    md1_cfg.int1_ff           = val.free_fall;
    md1_cfg.int1_wu           = val.wake_up;
    md1_cfg.int1_single_tap   = val.single_tap;
    md1_cfg.int1_sleep_change = val.sleep_change;

    emb_func_int1.int1_step_detector = val.step_detector;
    emb_func_int1.int1_tilt          = val.tilt;
    emb_func_int1.int1_sig_mot       = val.sig_mot;
    emb_func_int1.int1_fsm_lc        = val.fsm_lc;

    fsm_int1_a.int1_fsm1 = val.fsm1;
    fsm_int1_a.int1_fsm2 = val.fsm2;
    fsm_int1_a.int1_fsm3 = val.fsm3;
    fsm_int1_a.int1_fsm4 = val.fsm4;
    fsm_int1_a.int1_fsm5 = val.fsm5;
    fsm_int1_a.int1_fsm6 = val.fsm6;
    fsm_int1_a.int1_fsm7 = val.fsm7;
    fsm_int1_a.int1_fsm8 = val.fsm8;

    fsm_int1_b.int1_fsm9  = val.fsm9 ;
    fsm_int1_b.int1_fsm10 = val.fsm10;
    fsm_int1_b.int1_fsm11 = val.fsm11;
    fsm_int1_b.int1_fsm12 = val.fsm12;
    fsm_int1_b.int1_fsm13 = val.fsm13;
    fsm_int1_b.int1_fsm14 = val.fsm14;
    fsm_int1_b.int1_fsm15 = val.fsm15;
    fsm_int1_b.int1_fsm16 = val.fsm16;

    ctx->read_reg(ctx->handle, LSM6DSO_CTRL4_C, (uint8_t*)&ctrl4_c, 1);
    if( ( val.drdy_temp | val.timestamp ) != PROPERTY_DISABLE)
        ctrl4_c.int2_on_int1 = PROPERTY_ENABLE;
    else
        ctrl4_c.int2_on_int1 = PROPERTY_DISABLE;

    ctx->write_reg(ctx->handle, LSM6DSO_CTRL4_C, (uint8_t*)&ctrl4_c, 1);

    lsm6dso_mem_bank_set(ctx, LSM6DSO_EMBEDDED_FUNC_BANK);
    ctx->write_reg(ctx->handle, LSM6DSO_EMB_FUNC_INT1, (uint8_t*)&emb_func_int1, 1);
    ctx->write_reg(ctx->handle, LSM6DSO_FSM_INT1_A, (uint8_t*)&fsm_int1_a, 1);
    ctx->write_reg(ctx->handle, LSM6DSO_FSM_INT1_B, (uint8_t*)&fsm_int1_b, 1);

    lsm6dso_mem_bank_set(ctx, LSM6DSO_USER_BANK);

    if ( ( emb_func_int1.int1_fsm_lc
         | emb_func_int1.int1_sig_mot
         | emb_func_int1.int1_step_detector
         | emb_func_int1.int1_tilt
         | fsm_int1_a.int1_fsm1
         | fsm_int1_a.int1_fsm2
         | fsm_int1_a.int1_fsm3
         | fsm_int1_a.int1_fsm4
         | fsm_int1_a.int1_fsm5
         | fsm_int1_a.int1_fsm6
         | fsm_int1_a.int1_fsm7
         | fsm_int1_a.int1_fsm8
         | fsm_int1_b.int1_fsm9
         | fsm_int1_b.int1_fsm10
         | fsm_int1_b.int1_fsm11
         | fsm_int1_b.int1_fsm12
         | fsm_int1_b.int1_fsm13
         | fsm_int1_b.int1_fsm14
         | fsm_int1_b.int1_fsm15
         | fsm_int1_b.int1_fsm16) != PROPERTY_DISABLE)
    {
        md1_cfg.int1_emb_func = PROPERTY_ENABLE;
    }
    else{
        md1_cfg.int1_emb_func = PROPERTY_DISABLE;
    }

    ctx->write_reg(ctx->handle, LSM6DSO_INT1_CTRL, (uint8_t*)&int1_ctrl, 1);
    ctx->write_reg(ctx->handle, LSM6DSO_MD1_CFG, (uint8_t*)&md1_cfg, 1);

    ctx->read_reg(ctx->handle, LSM6DSO_INT2_CTRL, (uint8_t*)&int2_ctrl, 1);
    int2_ctrl.int2_drdy_temp = val.drdy_temp;
    ctx->write_reg(ctx->handle, LSM6DSO_INT2_CTRL, (uint8_t*)&int2_ctrl, 1);
    ctx->read_reg(ctx->handle, LSM6DSO_MD2_CFG, (uint8_t*)&md2_cfg, 1);
    md2_cfg.int2_timestamp = val.timestamp;
    ctx->write_reg(ctx->handle, LSM6DSO_MD2_CFG, (uint8_t*)&md2_cfg, 1);

    ctx->read_reg(ctx->handle, LSM6DSO_TAP_CFG2, (uint8_t*) &tap_cfg2, 1);
    lsm6dso_pin_int2_route_get(ctx, NULL, &pin_int2_route);
    if ( ( pin_int2_route.fifo_bdr
         | pin_int2_route.drdy_g
         | pin_int2_route.drdy_temp
         | pin_int2_route.drdy_xl
         | pin_int2_route.fifo_full
         | pin_int2_route.fifo_ovr
         | pin_int2_route.fifo_th
         | pin_int2_route.six_d
         | pin_int2_route.double_tap
         | pin_int2_route.free_fall
         | pin_int2_route.wake_up
         | pin_int2_route.single_tap
         | pin_int2_route.sleep_change
         | int1_ctrl.den_drdy_flag
         | int1_ctrl.int1_boot
         | int1_ctrl.int1_cnt_bdr
         | int1_ctrl.int1_drdy_g
         | int1_ctrl.int1_drdy_xl
         | int1_ctrl.int1_fifo_full
         | int1_ctrl.int1_fifo_ovr
         | int1_ctrl.int1_fifo_th
         | md1_cfg.int1_shub
         | md1_cfg.int1_6d
         | md1_cfg.int1_double_tap
         | md1_cfg.int1_ff
         | md1_cfg.int1_wu
         | md1_cfg.int1_single_tap
         | md1_cfg.int1_sleep_change) != PROPERTY_DISABLE) {
      tap_cfg2.interrupts_enable = PROPERTY_ENABLE;
    }
    else{
      tap_cfg2.interrupts_enable = PROPERTY_DISABLE;
    }
    ctx->write_reg(ctx->handle, LSM6DSO_TAP_CFG2, (uint8_t*) &tap_cfg2, 1);
}

/**
  * @brief  Route interrupt signals on int2 pin.[set]
  *
  * @param  ctx          communication interface handler. Use NULL to ingnore
  *                      this interface.(ptr)
  * @param  aux_ctx      auxiliary communication interface handler. Use NULL
  *                      to ingnore this interface.(ptr)
  * @param  val          the signals to route on int2 pin.
  *
  */
void lsm6dso_pin_int2_route_set(stmdev_ctx_t *ctx, stmdev_ctx_t *aux_ctx,
                                    lsm6dso_pin_int2_route_t val)
{
    lsm6dso_pin_int1_route_t pin_int1_route;
    lsm6dso_emb_func_int2_t  emb_func_int2;
    lsm6dso_fsm_int2_a_t     fsm_int2_a;
    lsm6dso_fsm_int2_b_t     fsm_int2_b;
    lsm6dso_int2_ctrl_t      int2_ctrl;
    lsm6dso_tap_cfg2_t       tap_cfg2;
    lsm6dso_md2_cfg_t        md2_cfg;
    lsm6dso_ctrl4_c_t        ctrl4_c;
    lsm6dso_int_ois_t        int_ois;

    if( aux_ctx != NULL )
    {
        aux_ctx->read_reg(aux_ctx->handle, LSM6DSO_INT_OIS, (uint8_t*)&int_ois, 1);
        int_ois.int2_drdy_ois = val.drdy_ois;
        aux_ctx->write_reg(aux_ctx->handle, LSM6DSO_INT_OIS, (uint8_t*)&int_ois, 1);
    }

    if( ctx != NULL )
    {
        int2_ctrl.int2_drdy_xl   = val.drdy_xl;
        int2_ctrl.int2_drdy_g    = val.drdy_g;
        int2_ctrl.int2_drdy_temp = val.drdy_temp;
        int2_ctrl.int2_fifo_th   = val.fifo_th;
        int2_ctrl.int2_fifo_ovr  = val.fifo_ovr;
        int2_ctrl.int2_fifo_full = val.fifo_full;
        int2_ctrl.int2_cnt_bdr   = val.fifo_bdr;

        md2_cfg.int2_timestamp    = val.timestamp;
        md2_cfg.int2_6d           = val.six_d;
        md2_cfg.int2_double_tap   = val.double_tap;
        md2_cfg.int2_ff           = val.free_fall;
        md2_cfg.int2_wu           = val.wake_up;
        md2_cfg.int2_single_tap   = val.single_tap;
        md2_cfg.int2_sleep_change = val.sleep_change;

        emb_func_int2. int2_step_detector = val.step_detector;
        emb_func_int2.int2_tilt           = val.tilt;
        emb_func_int2.int2_fsm_lc         = val.fsm_lc;

        fsm_int2_a.int2_fsm1 = val.fsm1;
        fsm_int2_a.int2_fsm2 = val.fsm2;
        fsm_int2_a.int2_fsm3 = val.fsm3;
        fsm_int2_a.int2_fsm4 = val.fsm4;
        fsm_int2_a.int2_fsm5 = val.fsm5;
        fsm_int2_a.int2_fsm6 = val.fsm6;
        fsm_int2_a.int2_fsm7 = val.fsm7;
        fsm_int2_a.int2_fsm8 = val.fsm8;

        fsm_int2_b.int2_fsm9  = val.fsm9 ;
        fsm_int2_b.int2_fsm10 = val.fsm10;
        fsm_int2_b.int2_fsm11 = val.fsm11;
        fsm_int2_b.int2_fsm12 = val.fsm12;
        fsm_int2_b.int2_fsm13 = val.fsm13;
        fsm_int2_b.int2_fsm14 = val.fsm14;
        fsm_int2_b.int2_fsm15 = val.fsm15;
        fsm_int2_b.int2_fsm16 = val.fsm16;

        ctx->read_reg(ctx->handle, LSM6DSO_CTRL4_C, (uint8_t*)&ctrl4_c, 1);
        if (( val.drdy_temp | val.timestamp ) != PROPERTY_DISABLE)
            ctrl4_c.int2_on_int1 = PROPERTY_DISABLE;
        else
            ctrl4_c.int2_on_int1 = PROPERTY_ENABLE;

        ctx->write_reg(ctx->handle, LSM6DSO_CTRL4_C, (uint8_t*)&ctrl4_c, 1);

        lsm6dso_mem_bank_set(ctx, LSM6DSO_EMBEDDED_FUNC_BANK);
        ctx->write_reg(ctx->handle, LSM6DSO_EMB_FUNC_INT2, (uint8_t*)&emb_func_int2, 1);
        ctx->write_reg(ctx->handle, LSM6DSO_FSM_INT2_A, (uint8_t*)&fsm_int2_a, 1);
        ctx->write_reg(ctx->handle, LSM6DSO_FSM_INT2_B, (uint8_t*)&fsm_int2_b, 1);

        lsm6dso_mem_bank_set(ctx, LSM6DSO_USER_BANK);

        if (( emb_func_int2.int2_fsm_lc
            | emb_func_int2.int2_sig_mot
            | emb_func_int2.int2_step_detector
            | emb_func_int2.int2_tilt
            | fsm_int2_a.int2_fsm1
            | fsm_int2_a.int2_fsm2
            | fsm_int2_a.int2_fsm3
            | fsm_int2_a.int2_fsm4
            | fsm_int2_a.int2_fsm5
            | fsm_int2_a.int2_fsm6
            | fsm_int2_a.int2_fsm7
            | fsm_int2_a.int2_fsm8
            | fsm_int2_b.int2_fsm9
            | fsm_int2_b.int2_fsm10
            | fsm_int2_b.int2_fsm11
            | fsm_int2_b.int2_fsm12
            | fsm_int2_b.int2_fsm13
            | fsm_int2_b.int2_fsm14
            | fsm_int2_b.int2_fsm15
            | fsm_int2_b.int2_fsm16)!= PROPERTY_DISABLE )
        {
            md2_cfg.int2_emb_func = PROPERTY_ENABLE;
        }
        else
        {
            md2_cfg.int2_emb_func = PROPERTY_DISABLE;
        }
        ctx->write_reg(ctx->handle, LSM6DSO_INT2_CTRL, (uint8_t*)&int2_ctrl, 1);
        ctx->write_reg(ctx->handle, LSM6DSO_MD2_CFG, (uint8_t*)&md2_cfg, 1);

        ctx->read_reg(ctx->handle, LSM6DSO_TAP_CFG2, (uint8_t*) &tap_cfg2, 1);

        lsm6dso_pin_int1_route_get(ctx, &pin_int1_route);

        if (( val.fifo_bdr
            | val.drdy_g
            | val.drdy_temp
            | val.drdy_xl
            | val.fifo_full
            | val.fifo_ovr
            | val.fifo_th
            | val.six_d
            | val.double_tap
            | val.free_fall
            | val.wake_up
            | val.single_tap
            | val.sleep_change
            | pin_int1_route.den_flag
            | pin_int1_route.boot
            | pin_int1_route.fifo_bdr
            | pin_int1_route.drdy_g
            | pin_int1_route.drdy_xl
            | pin_int1_route.fifo_full
            | pin_int1_route.fifo_ovr
            | pin_int1_route.fifo_th
            | pin_int1_route.six_d
            | pin_int1_route.double_tap
            | pin_int1_route.free_fall
            | pin_int1_route.wake_up
            | pin_int1_route.single_tap
            | pin_int1_route.sleep_change ) != PROPERTY_DISABLE)
        {
            tap_cfg2.interrupts_enable = PROPERTY_ENABLE;
        }
        else
        {
            tap_cfg2.interrupts_enable = PROPERTY_DISABLE;
        }
        ctx->write_reg(ctx->handle, LSM6DSO_TAP_CFG2, (uint8_t*) &tap_cfg2, 1);
    }
}

/**
  * @brief  Get the status of all the interrupt sources.[get]
  *
  * @param  ctx          communication interface handler.(ptr)
  * @param  val          the status of all the interrupt sources.(ptr)
  *
  */
void lsm6dso_all_sources_get(stmdev_ctx_t *ctx,
                                 lsm6dso_all_sources_t *val)
{
    lsm6dso_emb_func_status_mainpage_t emb_func_status_mainpage;
    lsm6dso_status_master_mainpage_t   status_master_mainpage;
    lsm6dso_fsm_status_a_mainpage_t    fsm_status_a_mainpage;
    lsm6dso_fsm_status_b_mainpage_t    fsm_status_b_mainpage;
    lsm6dso_fifo_status1_t             fifo_status1;
    lsm6dso_fifo_status2_t             fifo_status2;
    lsm6dso_all_int_src_t              all_int_src;
    lsm6dso_wake_up_src_t              wake_up_src;
    lsm6dso_status_reg_t               status_reg;
    lsm6dso_tap_src_t                  tap_src;
    lsm6dso_d6d_src_t                  d6d_src;
    lsm6dso_ctrl5_c_t                  ctrl5_c;
    uint8_t                            reg[12];

    ctx->read_reg(ctx->handle, LSM6DSO_CTRL5_C, (uint8_t*)&ctrl5_c, 1);
    ctrl5_c.not_used_01 = PROPERTY_ENABLE;
    ctx->write_reg(ctx->handle, LSM6DSO_CTRL5_C, (uint8_t*)&ctrl5_c, 1);

    ctx->read_reg(ctx->handle, LSM6DSO_ALL_INT_SRC, reg, 12);

    bytecpy(( uint8_t*)&all_int_src, &reg[0]);
    bytecpy(( uint8_t*)&wake_up_src, &reg[1]);
    bytecpy(( uint8_t*)&tap_src, &reg[2]);
    bytecpy(( uint8_t*)&d6d_src, &reg[3]);
    bytecpy(( uint8_t*)&status_reg, &reg[4]);
    bytecpy(( uint8_t*)&emb_func_status_mainpage, &reg[5]);
    bytecpy(( uint8_t*)&fsm_status_a_mainpage, &reg[6]);
    bytecpy(( uint8_t*)&fsm_status_b_mainpage, &reg[7]);
    bytecpy(( uint8_t*)&status_master_mainpage, &reg[9]);
    bytecpy(( uint8_t*)&fifo_status1, &reg[10]);
    bytecpy(( uint8_t*)&fifo_status2, &reg[11]);

    val->timestamp = all_int_src.timestamp_endcount;

    val->wake_up_z    = wake_up_src.z_wu;
    val->wake_up_y    = wake_up_src.y_wu;
    val->wake_up_x    = wake_up_src.x_wu;
    val->wake_up      = wake_up_src.wu_ia;
    val->sleep_state  = wake_up_src.sleep_state;
    val->free_fall    = wake_up_src.ff_ia;
    val->sleep_change = wake_up_src.sleep_change_ia;

    val->tap_x      = tap_src.x_tap;
    val->tap_y      = tap_src.y_tap;
    val->tap_z      = tap_src.z_tap;
    val->tap_sign   = tap_src.tap_sign;
    val->double_tap = tap_src.double_tap;
    val->single_tap = tap_src.single_tap;

    val->six_d_xl = d6d_src.xl;
    val->six_d_xh = d6d_src.xh;
    val->six_d_yl = d6d_src.yl;
    val->six_d_yh = d6d_src.yh;
    val->six_d_zl = d6d_src.zl;
    val->six_d_zh = d6d_src.zh;
    val->six_d    = d6d_src.d6d_ia;
    val->den_flag = d6d_src.den_drdy;

    val->drdy_xl   = status_reg.xlda;
    val->drdy_g    = status_reg.gda;
    val->drdy_temp = status_reg.tda;

    val->step_detector = emb_func_status_mainpage.is_step_det;
    val->tilt          = emb_func_status_mainpage.is_tilt;
    val->sig_mot       = emb_func_status_mainpage.is_sigmot;
    val->fsm_lc        = emb_func_status_mainpage.is_fsm_lc;

    val->fsm1 = fsm_status_a_mainpage.is_fsm1;
    val->fsm2 = fsm_status_a_mainpage.is_fsm2;
    val->fsm3 = fsm_status_a_mainpage.is_fsm3;
    val->fsm4 = fsm_status_a_mainpage.is_fsm4;
    val->fsm5 = fsm_status_a_mainpage.is_fsm5;
    val->fsm6 = fsm_status_a_mainpage.is_fsm6;
    val->fsm7 = fsm_status_a_mainpage.is_fsm7;
    val->fsm8 = fsm_status_a_mainpage.is_fsm8;

    val->fsm9  = fsm_status_b_mainpage.is_fsm9;
    val->fsm10 = fsm_status_b_mainpage.is_fsm10;
    val->fsm11 = fsm_status_b_mainpage.is_fsm11;
    val->fsm12 = fsm_status_b_mainpage.is_fsm12;
    val->fsm13 = fsm_status_b_mainpage.is_fsm13;
    val->fsm14 = fsm_status_b_mainpage.is_fsm14;
    val->fsm15 = fsm_status_b_mainpage.is_fsm15;
    val->fsm16 = fsm_status_b_mainpage.is_fsm16;

    val->sh_endop       = status_master_mainpage.sens_hub_endop;
    val->sh_slave0_nack = status_master_mainpage.slave0_nack;
    val->sh_slave1_nack = status_master_mainpage.slave1_nack;
    val->sh_slave2_nack = status_master_mainpage.slave2_nack;
    val->sh_slave3_nack = status_master_mainpage.slave3_nack;
    val->sh_wr_once     = status_master_mainpage.wr_once_done;

    val->fifo_diff = (256U * fifo_status2.diff_fifo) + fifo_status1.diff_fifo;

    val->fifo_ovr_latched = fifo_status2.over_run_latched;
    val->fifo_bdr         = fifo_status2.counter_bdr_ia;
    val->fifo_full        = fifo_status2.fifo_full_ia;
    val->fifo_ovr         = fifo_status2.fifo_ovr_ia;
    val->fifo_th          = fifo_status2.fifo_wtm_ia;

    ctrl5_c.not_used_01 = PROPERTY_DISABLE;
    ctx->write_reg(ctx->handle, LSM6DSO_CTRL5_C, (uint8_t*)&ctrl5_c, 1);
}

/**
  * @brief  Accelerometer new data available.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of xlda in reg STATUS_REG
  *
  */
void lsm6dso_xl_flag_data_ready_get(stmdev_ctx_t *ctx, uint8_t *val)
{
    lsm6dso_status_reg_t reg;

    ctx->read_reg(ctx->handle, LSM6DSO_STATUS_REG, (uint8_t*)&reg, 1);
    *val = reg.xlda;
}

/**
  * @brief  Gyroscope new data available.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of gda in reg STATUS_REG
  *
  */
void lsm6dso_gy_flag_data_ready_get(stmdev_ctx_t *ctx, uint8_t *val)
{
    lsm6dso_status_reg_t reg;

    ctx->read_reg(ctx->handle, LSM6DSO_STATUS_REG, (uint8_t*)&reg, 1);
    *val = reg.gda;
}

/**
  * @brief  Angular rate sensor. The value is expressed as a 16-bit
  *         word in twos complement.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  buff     buffer that stores data read
  *
  */
void lsm6dso_angular_rate_raw_get(stmdev_ctx_t *ctx, uint8_t *buff)
{
    ctx->read_reg(ctx->handle, LSM6DSO_OUTX_L_G, buff, 6);
}

/**
  * @brief  Linear acceleration output register.
  *         The value is expressed as a 16-bit word in two's complement.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  buff     buffer that stores data read
  *
  */
void lsm6dso_acceleration_raw_get(stmdev_ctx_t *ctx, uint8_t *buff)
{
    ctx->read_reg(ctx->handle, LSM6DSO_OUTX_L_A, buff, 6);
}

// ISR callback for the host IOINT
static void lsm_int1_handler(void)
{
    apollo_irq.lsm_irq1 =  1;
}

static void lsm_int2_handler(void)
{
    pr_err("irq2\n");
    //apollo_irq.lsm_irq2 =  1;
}

/**
  * @brief  Data-ready pulsed / letched mode.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of
  *                                     dataready_pulsed in
  *                                     reg COUNTER_BDR_REG1
  *
  */
void lsm6dso_data_ready_mode_set(stmdev_ctx_t *ctx,
                                    lsm6dso_dataready_pulsed_t val)
{
    lsm6dso_counter_bdr_reg1_t reg;

    ctx->read_reg(ctx->handle, LSM6DSO_COUNTER_BDR_REG1, (uint8_t*)&reg, 1);
    reg.dataready_pulsed = (uint8_t)val;
    ctx->write_reg(ctx->handle, LSM6DSO_COUNTER_BDR_REG1, (uint8_t*)&reg, 1);
}

/**
  * @brief  Reset step counter register.[get]
  *
  * @param  ctx      read / write interface definitions
  *
  */
void lsm6dso_steps_reset(stmdev_ctx_t *ctx)
{
    lsm6dso_emb_func_src_t reg;

    lsm6dso_mem_bank_set(ctx, LSM6DSO_EMBEDDED_FUNC_BANK);
    ctx->read_reg(ctx->handle, LSM6DSO_EMB_FUNC_SRC, (uint8_t*)&reg, 1);
    reg.pedo_rst_step = PROPERTY_ENABLE;
    ctx->write_reg(ctx->handle, LSM6DSO_EMB_FUNC_SRC, (uint8_t*)&reg, 1);
    lsm6dso_mem_bank_set(ctx, LSM6DSO_USER_BANK);
}

/**
  * @brief  Read a line(byte) in a page.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  uint8_t address: page line address
  * @param  val      read value
  *
  */
void lsm6dso_ln_pg_read_byte(stmdev_ctx_t *ctx, uint16_t address,
                                uint8_t *val)
{
    lsm6dso_page_rw_t page_rw;
    lsm6dso_page_sel_t page_sel;
    lsm6dso_page_address_t  page_address;

    lsm6dso_mem_bank_set(ctx, LSM6DSO_EMBEDDED_FUNC_BANK);

    ctx->read_reg(ctx->handle, LSM6DSO_PAGE_RW, (uint8_t*) &page_rw, 1);
    page_rw.page_rw = 0x01; /* page_read enable*/
    ctx->write_reg(ctx->handle, LSM6DSO_PAGE_RW, (uint8_t*) &page_rw, 1);

    ctx->read_reg(ctx->handle, LSM6DSO_PAGE_SEL, (uint8_t*) &page_sel, 1);
    page_sel.page_sel = ((uint8_t)(address >> 8) & 0x0FU);
    page_sel.not_used_01 = 1;
    ctx->write_reg(ctx->handle, LSM6DSO_PAGE_SEL, (uint8_t*) &page_sel, 1);
    page_address.page_addr = (uint8_t)address & 0x00FFU;
    ctx->write_reg(ctx->handle, LSM6DSO_PAGE_ADDRESS, (uint8_t*)&page_address, 1);

    ctx->read_reg(ctx->handle, LSM6DSO_PAGE_VALUE, val, 1);

    ctx->read_reg(ctx->handle, LSM6DSO_PAGE_RW, (uint8_t*) &page_rw, 1);
    page_rw.page_rw = 0x00; /* page_read disable */
    ctx->write_reg(ctx->handle, LSM6DSO_PAGE_RW, (uint8_t*) &page_rw, 1);
    lsm6dso_mem_bank_set(ctx, LSM6DSO_USER_BANK);
}

/**
  * @brief  Write a line(byte) in a page.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  uint8_t address: page line address
  * @param  val      value to write
  *
  */
void lsm6dso_ln_pg_write_byte(stmdev_ctx_t *ctx, uint16_t address,
                                 uint8_t *val)
{
    lsm6dso_page_rw_t page_rw;
    lsm6dso_page_sel_t page_sel;
    lsm6dso_page_address_t page_address;

    lsm6dso_mem_bank_set(ctx, LSM6DSO_EMBEDDED_FUNC_BANK);

    ctx->read_reg(ctx->handle, LSM6DSO_PAGE_RW, (uint8_t*) &page_rw, 1);
    page_rw.page_rw = 0x02; /* page_write enable */
    ctx->write_reg(ctx->handle, LSM6DSO_PAGE_RW, (uint8_t*) &page_rw, 1);

    ctx->read_reg(ctx->handle, LSM6DSO_PAGE_SEL, (uint8_t*) &page_sel, 1);
    page_sel.page_sel = ((uint8_t)(address >> 8) & 0x0FU);
    page_sel.not_used_01 = 1;
    ctx->write_reg(ctx->handle, LSM6DSO_PAGE_SEL, (uint8_t*) &page_sel, 1);

    page_address.page_addr = (uint8_t)address & 0xFFU;
    ctx->write_reg(ctx->handle, LSM6DSO_PAGE_ADDRESS, (uint8_t*)&page_address, 1);

    ctx->write_reg(ctx->handle, LSM6DSO_PAGE_VALUE, val, 1);
    ctx->read_reg(ctx->handle, LSM6DSO_PAGE_RW, (uint8_t*) &page_rw, 1);
    page_rw.page_rw = 0x00; /* page_write disable */
    ctx->write_reg(ctx->handle, LSM6DSO_PAGE_RW, (uint8_t*) &page_rw, 1);

    lsm6dso_mem_bank_set(ctx, LSM6DSO_USER_BANK);
}

/**
  * @brief  Enable pedometer algorithm.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      turn on and configure pedometer
  *
  */
void lsm6dso_pedo_sens_set(stmdev_ctx_t *ctx, lsm6dso_pedo_md_t val)
{
    lsm6dso_emb_func_en_a_t emb_func_en_a;
    lsm6dso_emb_func_en_b_t emb_func_en_b;
    lsm6dso_pedo_cmd_reg_t pedo_cmd_reg;

    lsm6dso_ln_pg_read_byte(ctx, LSM6DSO_PEDO_CMD_REG, (uint8_t*)&pedo_cmd_reg);
    lsm6dso_mem_bank_set(ctx, LSM6DSO_EMBEDDED_FUNC_BANK);
    ctx->read_reg(ctx->handle, LSM6DSO_EMB_FUNC_EN_A, (uint8_t*)&emb_func_en_a, 1);
    ctx->read_reg(ctx->handle, LSM6DSO_EMB_FUNC_EN_B, (uint8_t*)&emb_func_en_b, 1);

    emb_func_en_a.pedo_en = (uint8_t)val & 0x01U;
    emb_func_en_b.pedo_adv_en = ((uint8_t)val & 0x02U)>>1;
    pedo_cmd_reg.fp_rejection_en = ((uint8_t)val & 0x10U)>>4;
    pedo_cmd_reg.ad_det_en = ((uint8_t)val & 0x20U)>>5;
    ctx->write_reg(ctx->handle, LSM6DSO_EMB_FUNC_EN_A, (uint8_t*)&emb_func_en_a, 1);
    ctx->write_reg(ctx->handle, LSM6DSO_EMB_FUNC_EN_B, (uint8_t*)&emb_func_en_b, 1);
    lsm6dso_mem_bank_set(ctx, LSM6DSO_USER_BANK);

    lsm6dso_ln_pg_write_byte(ctx, LSM6DSO_PEDO_CMD_REG, (uint8_t*)&pedo_cmd_reg);
}

/**
  * @brief  Step counter output register.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  buff     buffer that stores data read
  *
  */
void lsm6dso_number_of_steps_get(stmdev_ctx_t *ctx, uint8_t *buff)
{
    lsm6dso_mem_bank_set(ctx, LSM6DSO_EMBEDDED_FUNC_BANK);
    ctx->read_reg(ctx->handle, LSM6DSO_STEP_COUNTER_L, buff, 2);
    lsm6dso_mem_bank_set(ctx, LSM6DSO_USER_BANK);
}

/**
  * @brief  Weight of XL user offset bits of registers X_OFS_USR (73h),
  *         Y_OFS_USR (74h), Z_OFS_USR (75h).[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of usr_off_w in reg CTRL6_C
  *
  */
void lsm6dso_xl_offset_weight_set(stmdev_ctx_t *ctx,
                                     lsm6dso_usr_off_w_t val)
{
    lsm6dso_ctrl6_c_t reg;

    ctx->read_reg(ctx->handle, LSM6DSO_CTRL6_C, (uint8_t*)&reg, 1);
    reg.usr_off_w = (uint8_t)val;
    ctx->write_reg(ctx->handle, LSM6DSO_CTRL6_C, (uint8_t*)&reg, 1);
}

/**
  * @brief  Accelerometer X-axis user offset correction expressed in
  *         two's complement, weight depends on USR_OFF_W in CTRL6_C (15h).
  *         The value must be in the range [-127 127].[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  buff     buffer that contains data to write
  *
  */
void lsm6dso_xl_usr_offset_x_set(stmdev_ctx_t *ctx, uint8_t *buff)
{
    ctx->write_reg(ctx->handle, LSM6DSO_X_OFS_USR, buff, 1);
}

/**
  * @brief  Accelerometer X-axis user offset correction expressed in two’s
  *         complement, weight depends on USR_OFF_W in CTRL6_C (15h).
  *         The value must be in the range [-127 127].[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  buff     buffer that stores data read
  *
  */
void lsm6dso_xl_usr_offset_x_get(stmdev_ctx_t *ctx, uint8_t *buff)
{
    ctx->read_reg(ctx->handle, LSM6DSO_X_OFS_USR, buff, 1);
}

/**
  * @brief  Accelerometer Y-axis user offset correction expressed in two’s
  *         complement, weight depends on USR_OFF_W in CTRL6_C (15h).
  *         The value must be in the range [-127 127].[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  buff     buffer that contains data to write
  *
  */
void lsm6dso_xl_usr_offset_y_set(stmdev_ctx_t *ctx, uint8_t *buff)
{
    ctx->write_reg(ctx->handle, LSM6DSO_Y_OFS_USR, buff, 1);
}

/**
  * @brief  Accelerometer Y-axis user offset correction expressed in two’s
  *         complement, weight depends on USR_OFF_W in CTRL6_C (15h).
  *         The value must be in the range [-127 127].[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  buff     buffer that stores data read
  *
  */
void lsm6dso_xl_usr_offset_y_get(stmdev_ctx_t *ctx, uint8_t *buff)
{
    ctx->read_reg(ctx->handle, LSM6DSO_Y_OFS_USR, buff, 1);
}

/**
  * @brief  Accelerometer Z-axis user offset correction expressed in two's
  *         complement, weight depends on USR_OFF_W in CTRL6_C (15h).
  *         The value must be in the range [-127 127].[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  buff     buffer that contains data to write
  *
  */
void lsm6dso_xl_usr_offset_z_set(stmdev_ctx_t *ctx, uint8_t *buff)
{
    ctx->write_reg(ctx->handle, LSM6DSO_Z_OFS_USR, buff, 1);
}

/**
  * @brief  Accelerometer Z-axis user offset correction expressed in two’s
  *         complement, weight depends on USR_OFF_W in CTRL6_C (15h).
  *         The value must be in the range [-127 127].[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  buff     buffer that stores data read
  *
  */
void lsm6dso_xl_usr_offset_z_get(stmdev_ctx_t *ctx, uint8_t *buff)
{
    ctx->read_reg(ctx->handle, LSM6DSO_Z_OFS_USR, buff, 1);
}

/**
  * @brief  Enables user offset on out.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of usr_off_on_out in reg CTRL7_G
  *
  */
void lsm6dso_xl_usr_offset_set(stmdev_ctx_t *ctx, uint8_t val)
{
    lsm6dso_ctrl7_g_t reg;

    ctx->read_reg(ctx->handle, LSM6DSO_CTRL7_G, (uint8_t*)&reg, 1);
    reg.usr_off_on_out = val;
    ctx->write_reg(ctx->handle, LSM6DSO_CTRL7_G, (uint8_t*)&reg, 1);
}

/**
  * @brief  User offset on out flag.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      values of usr_off_on_out in reg CTRL7_G
  *
  */
void lsm6dso_xl_usr_offset_get(stmdev_ctx_t *ctx, uint8_t *val)
{
    lsm6dso_ctrl7_g_t reg;

    ctx->read_reg(ctx->handle, LSM6DSO_CTRL7_G, (uint8_t*)&reg, 1);
    *val = reg.usr_off_on_out;
}

/**
  * @brief  Enable tilt calculation.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of tilt_en in reg EMB_FUNC_EN_A
  *
  */
void lsm6dso_tilt_sens_set(stmdev_ctx_t *ctx, uint8_t val)
{
    lsm6dso_emb_func_en_a_t reg;

    lsm6dso_mem_bank_set(ctx, LSM6DSO_EMBEDDED_FUNC_BANK);
    ctx->read_reg(ctx->handle, LSM6DSO_EMB_FUNC_EN_A, (uint8_t*)&reg, 1);
    reg.tilt_en = val;
    ctx->write_reg(ctx->handle, LSM6DSO_EMB_FUNC_EN_A, (uint8_t*)&reg, 1);
    lsm6dso_mem_bank_set(ctx, LSM6DSO_USER_BANK);
}

/**
  * @brief  Interrupt status bit for tilt detection.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of is_tilt in reg EMB_FUNC_STATUS
  *
  */
void lsm6dso_tilt_flag_data_ready_get(stmdev_ctx_t *ctx, uint8_t *val)
{
    lsm6dso_emb_func_status_t reg;

    lsm6dso_mem_bank_set(ctx, LSM6DSO_EMBEDDED_FUNC_BANK);
    ctx->read_reg(ctx->handle, LSM6DSO_EMB_FUNC_STATUS, (uint8_t*)&reg, 1);
    *val = reg.is_tilt;
    lsm6dso_mem_bank_set(ctx, LSM6DSO_USER_BANK);
}

//*****************************************************************************
//
//! @brief Configures the necessary pins for lsm6dso
//!
//! This function configures a SPI to drive lsm6dso.
//!
//! @return None.
//
//*****************************************************************************
void lsm6dso_init(void)
{
    uint8_t whoAmI = 0;
    uint8_t rst;
    //lsm6dso_pin_int1_route_t int1_route;
    //lsm6dso_pin_int2_route_t int2_route;


    am_hal_iom_config_t m_sIOMSpiConfig =
    {
        .eInterfaceMode = AM_HAL_IOM_SPI_MODE,
        .ui32ClockFreq = AM_HAL_IOM_8MHZ,
        .eSpiMode = AM_HAL_IOM_SPI_MODE_0,
    };

    am_hal_gpio_pincfg_t m_lsm6dsoGpioInt1 =
    {
        .uFuncSel = AM_HAL_PIN_14_GPIO,
        .eDriveStrength = AM_HAL_GPIO_PIN_DRIVESTRENGTH_2MA,
        .eIntDir = AM_HAL_GPIO_PIN_INTDIR_LO2HI,
        .eGPInput = AM_HAL_GPIO_PIN_INPUT_ENABLE,
    };

    am_hal_gpio_pincfg_t m_lsm6dsoGpioInt2 =
    {
        .uFuncSel = AM_HAL_PIN_15_GPIO,
        .eDriveStrength = AM_HAL_GPIO_PIN_DRIVESTRENGTH_2MA,
        .eIntDir = AM_HAL_GPIO_PIN_INTDIR_LO2HI,
        .eGPInput = AM_HAL_GPIO_PIN_INPUT_ENABLE,
    };

    am_hal_iom_initialize(LSM6DSO_IOM_MODULE, &g_Lsm6dsoCtx.handle);
    am_hal_iom_power_ctrl(g_Lsm6dsoCtx.handle, AM_HAL_SYSCTRL_WAKE, false);

    // Set the required configuration settings for the IOM.
    am_hal_iom_configure(g_Lsm6dsoCtx.handle, &m_sIOMSpiConfig);

    g_Lsm6dsoCtx.read_reg = lsm6d_read;
    g_Lsm6dsoCtx.write_reg = lsm6d_write;

    // Configure the IOM pins.
    am_bsp_iom_pins_enable(LSM6DSO_IOM_MODULE, AM_HAL_IOM_SPI_MODE);
    if (sw_version[0] == 0)
    {
        am_hal_gpio_state_write(LSM6DSO_PIN_CE, AM_HAL_GPIO_OUTPUT_SET);
        am_hal_gpio_pinconfig(LSM6DSO_PIN_CE, g_AM_HAL_GPIO_OUTPUT_8);
    }
    else
    {
        am_hal_gpio_pinconfig(AM_BSP_GPIO_IOM0_CS, g_AM_BSP_GPIO_IOM0_CS);
    }

    // Enable the IOM.
    am_hal_iom_enable(g_Lsm6dsoCtx.handle);
    am_util_delay_ms(1);

    // GPIO irq set up
    am_hal_gpio_pinconfig(LSM6DSO_PIN_INT1, m_lsm6dsoGpioInt1);
    am_hal_gpio_pinconfig(LSM6DSO_PIN_INT2, m_lsm6dsoGpioInt2);
    AM_HAL_GPIO_MASKCREATE(GpioIntMask);
    am_hal_gpio_interrupt_clear(AM_HAL_GPIO_MASKBIT(pGpioIntMask, LSM6DSO_PIN_INT1));
    am_hal_gpio_interrupt_clear(AM_HAL_GPIO_MASKBIT(pGpioIntMask, LSM6DSO_PIN_INT2));
    am_hal_gpio_interrupt_register(LSM6DSO_PIN_INT1, lsm_int1_handler);
    am_hal_gpio_interrupt_register(LSM6DSO_PIN_INT2, lsm_int2_handler);
    am_hal_gpio_interrupt_enable(AM_HAL_GPIO_MASKBIT(pGpioIntMask, LSM6DSO_PIN_INT1));
    am_hal_gpio_interrupt_enable(AM_HAL_GPIO_MASKBIT(pGpioIntMask, LSM6DSO_PIN_INT2));
    NVIC_EnableIRQ(GPIO_IRQn);

    /* Check device ID */
    lsm6dso_device_id_get(&g_Lsm6dsoCtx, &whoAmI);
    if(LSM6DSO_WHO_AM_I != (uint8_t)whoAmI)
    {
        PR_ERR("ERROR: lsm6dso get ID: 0x%02x error.", whoAmI);
        return;
    }
    else
        PR_INFO("lsm6dso get ID success.");

    lsm6dso_reset_set(&g_Lsm6dsoCtx, PROPERTY_ENABLE);
    do
    {
        lsm6dso_reset_get(&g_Lsm6dsoCtx, &rst);
    }
    while(rst);

    /* Disable I3C interface */
    lsm6dso_i3c_disable_set(&g_Lsm6dsoCtx, LSM6DSO_I3C_DISABLE);

    /* Enable Block Data Update */
    lsm6dso_block_data_update_set(&g_Lsm6dsoCtx, PROPERTY_ENABLE);

    /* Weight of XL user offset to 2^(-10) g/LSB */
    lsm6dso_xl_offset_weight_set(&g_Lsm6dsoCtx, LSM6DSO_LSb_1mg);
    lsm6dso_xl_usr_offset_set(&g_Lsm6dsoCtx, PROPERTY_DISABLE);

    /* Set XL and Gyro Output Data Rate */
    lsm6dso_xl_data_rate_set(&g_Lsm6dsoCtx, LSM6DSO_XL_ODR_26Hz);
    lsm6dso_gy_data_rate_set(&g_Lsm6dsoCtx, LSM6DSO_GY_ODR_12Hz5);

    /* Set 2g full XL scale and 2000 dps full Gyro */
    lsm6dso_xl_full_scale_set(&g_Lsm6dsoCtx, LSM6DSO_2g);
    lsm6dso_gy_full_scale_set(&g_Lsm6dsoCtx, LSM6DSO_2000dps);

    /* Reset steps of pedometer */
    lsm6dso_steps_reset(&g_Lsm6dsoCtx);

    /* Enable pedometer */
    lsm6dso_pedo_sens_set(&g_Lsm6dsoCtx, LSM6DSO_FALSE_STEP_REJ_ADV_MODE);

    /* Enable Tilt in embedded function. */
    lsm6dso_tilt_sens_set(&g_Lsm6dsoCtx, PROPERTY_ENABLE);

    /* Disable interrupt generation on INT pin */
    lsm6dso_int1_route_set(LSM6DSO_int1_all, 0);
    lsm6dso_int2_route_set(LSM6DSO_int2_all, 0);
}

/**
  * @brief  Enable access to the embedded functions/sensor
  *         hub configuration registers.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of reg_access in
  *                               reg FUNC_CFG_ACCESS
  *
  */
void lsm6dso_mem_bank_set(stmdev_ctx_t *ctx, lsm6dso_reg_access_t val)
{
    lsm6dso_func_cfg_access_t reg;

    ctx->read_reg(ctx->handle, LSM6DSO_FUNC_CFG_ACCESS, (uint8_t*)&reg, 1);
    reg.reg_access = (uint8_t)val;
    ctx->write_reg(ctx->handle, LSM6DSO_FUNC_CFG_ACCESS, (uint8_t*)&reg, 1);
}

/**
  * @brief  Final State Machine enable.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      union of registers from FSM_ENABLE_A to FSM_ENABLE_B
  *
  */
void lsm6dso_fsm_enable_get(stmdev_ctx_t *ctx,
                               lsm6dso_emb_fsm_enable_t *val)
{
    lsm6dso_mem_bank_set(ctx, LSM6DSO_EMBEDDED_FUNC_BANK);
    ctx->read_reg(ctx->handle, LSM6DSO_FSM_ENABLE_A, (uint8_t*)val, 2);
    lsm6dso_mem_bank_set(ctx, LSM6DSO_USER_BANK);
}

/**
  * @brief  Finite State Machine ODR configuration.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Get the values of fsm_odr in reg EMB_FUNC_ODR_CFG_B
  *
  */
void lsm6dso_fsm_data_rate_get(stmdev_ctx_t *ctx, lsm6dso_fsm_odr_t *val)
{
    lsm6dso_emb_func_odr_cfg_b_t reg;

    lsm6dso_mem_bank_set(ctx, LSM6DSO_EMBEDDED_FUNC_BANK);
    ctx->read_reg(ctx->handle, LSM6DSO_EMB_FUNC_ODR_CFG_B, (uint8_t*)&reg, 1);
    switch (reg.fsm_odr) {
      case LSM6DSO_ODR_FSM_12Hz5:
        *val = LSM6DSO_ODR_FSM_12Hz5;
        break;
      case LSM6DSO_ODR_FSM_26Hz:
        *val = LSM6DSO_ODR_FSM_26Hz;
        break;
      case LSM6DSO_ODR_FSM_52Hz:
        *val = LSM6DSO_ODR_FSM_52Hz;
        break;
      case LSM6DSO_ODR_FSM_104Hz:
        *val = LSM6DSO_ODR_FSM_104Hz;
        break;
      default:
        *val = LSM6DSO_ODR_FSM_12Hz5;
        break;
    }

    lsm6dso_mem_bank_set(ctx, LSM6DSO_USER_BANK);
}

void lsm6dso_int1_route_set(lsm6dso_int1_type_t int1_type, bool int1_val)
{
    lsm6dso_pin_int1_route_t m_int1_route;
    uint8_t* p_data;

    if (int1_val)
    {
        /* Enable drdy 75 us pulse: if interrupt must be pulsed */
        lsm6dso_data_ready_mode_set(&g_Lsm6dsoCtx, LSM6DSO_DRDY_PULSED);
    }

    if (int1_type == LSM6DSO_int1_all)
    {
        memset(&m_int1_route, int1_val, sizeof(m_int1_route));
        lsm6dso_pin_int1_route_set(&g_Lsm6dsoCtx, m_int1_route);

        return;
    }

    lsm6dso_pin_int1_route_get(&g_Lsm6dsoCtx, &m_int1_route);
    p_data = (uint8_t*)&m_int1_route + (int1_type / 8);
    if (int1_val)
        *p_data |= 1 << (int1_type % 8);
    else
        *p_data &= ~(1 << (int1_type % 8));

    lsm6dso_pin_int1_route_set(&g_Lsm6dsoCtx, m_int1_route);
}

void lsm6dso_int2_route_set(lsm6dso_int2_type_t int2_type, bool int2_val)
{
    lsm6dso_pin_int2_route_t m_int2_route;
    uint8_t* p_data;

    if (int2_val)
    {
        /* Enable drdy 75 us pulse: if interrupt must be pulsed */
        lsm6dso_data_ready_mode_set(&g_Lsm6dsoCtx, LSM6DSO_DRDY_PULSED);
    }

    if (int2_type == LSM6DSO_int2_all)
    {
        memset(&m_int2_route, int2_val, sizeof(m_int2_route));
        lsm6dso_pin_int2_route_set(&g_Lsm6dsoCtx, NULL, m_int2_route);

        return;
    }

    lsm6dso_pin_int2_route_get(&g_Lsm6dsoCtx, NULL, &m_int2_route);
    p_data = (uint8_t*)&m_int2_route + (int2_type / 8);
    if (int2_val)
        *p_data |= 1 << (int2_type % 8);
    else
        *p_data &= ~(1 << (int2_type % 8));

    lsm6dso_pin_int2_route_set(&g_Lsm6dsoCtx, NULL, m_int2_route);
}

// External call function
uint8_t lsm6dso_acceleration_get(float* acc_data)
{
    uint8_t reg = 0;

    if (acc_data == NULL)
        return 1;

    /* Read output only if new xl value is available */
    lsm6dso_xl_flag_data_ready_get(&g_Lsm6dsoCtx, &reg);
    if (reg)
    {
        /* Read acceleration field data */
        memset(data_raw_acceleration.u8bit, 0x00, 3 * sizeof(int16_t));
        lsm6dso_acceleration_raw_get(&g_Lsm6dsoCtx, data_raw_acceleration.u8bit);
        acc_data[0] = lsm6dso_from_fs2_to_mg(data_raw_acceleration.i16bit[0]);
        acc_data[1] = lsm6dso_from_fs2_to_mg(data_raw_acceleration.i16bit[1]);
        acc_data[2] = lsm6dso_from_fs2_to_mg(data_raw_acceleration.i16bit[2]);

        return 0;
    }
    else
    {
        return 2;
    }
}

uint8_t lsm6dso_angular_get(float* gyro_data)
{
    uint8_t reg = 0;

    if (gyro_data == NULL)
        return 1;

    /* Read output only if new gyro value is available */
    lsm6dso_gy_flag_data_ready_get(&g_Lsm6dsoCtx, &reg);
    if (reg)
    {
        /* Read angular rate field data */
        memset(data_raw_angular_rate.u8bit, 0x00, 3 * sizeof(int16_t));
        lsm6dso_angular_rate_raw_get(&g_Lsm6dsoCtx, data_raw_angular_rate.u8bit);
        gyro_data[0] = lsm6dso_from_fs2000_to_mdps(data_raw_angular_rate.i16bit[0]);
        gyro_data[1] = lsm6dso_from_fs2000_to_mdps(data_raw_angular_rate.i16bit[1]);
        gyro_data[2] = lsm6dso_from_fs2000_to_mdps(data_raw_angular_rate.i16bit[2]);

        return 0;
    }
    else
    {
        return 2;
    }
}

uint16_t lsm6dso_step_get(void)
{
    uint16_t step_ret;

    lsm6dso_number_of_steps_get(&g_Lsm6dsoCtx, (uint8_t*)&step_ret);

    return step_ret;
}

uint8_t lsm6dso_acc_cali(void)
{
    float acc_cali_data[128][3];
    float acc_total_data[3] = {0};
    int8_t acc_offset[3] = {0};
    uint8_t abnormal_num = 0;
    int16_t i;

    lsm6dso_xl_usr_offset_set(&g_Lsm6dsoCtx, PROPERTY_DISABLE);

    for(i = 0; i < 128; i++)
    {
        while (lsm6dso_acceleration_get(acc_cali_data[i]))
            am_util_delay_ms(2);

        if (acc_cali_data[i][0] < -50 || acc_cali_data[i][0] > 50 ||
            acc_cali_data[i][1] < -50 || acc_cali_data[i][1] > 50 ||
            acc_cali_data[i][2] < 950 || acc_cali_data[i][2] > 1050)
        {
            abnormal_num++;
            i--;
            continue;
        }
        if (abnormal_num > 2)
        {
            PR_ERR("Accelerometer calibration failed: Place horizontally.");
            return 1;
        }

        acc_total_data[0] += acc_cali_data[i][0];
        acc_total_data[1] += acc_cali_data[i][1];
        acc_total_data[2] += (acc_cali_data[i][2] - 1000);
    }

    acc_offset[0] = (int8_t)(acc_total_data[0] / 128);
    acc_offset[1] = (int8_t)(acc_total_data[1] / 128);
    acc_offset[2] = (int8_t)(acc_total_data[2] / 128);

    /* Accelerometer X,Y,Z axis user offset correction expressed
    * in two's complement.
    */
    lsm6dso_xl_usr_offset_x_set(&g_Lsm6dsoCtx, (uint8_t*)&acc_offset[0]);
    lsm6dso_xl_usr_offset_y_set(&g_Lsm6dsoCtx, (uint8_t*)&acc_offset[1]);
    lsm6dso_xl_usr_offset_z_set(&g_Lsm6dsoCtx, (uint8_t*)&acc_offset[2]);
    PR_INFO("ACC offset: %d, %d, %d", acc_offset[0], acc_offset[1], acc_offset[2]);
    lsm6dso_xl_usr_offset_set(&g_Lsm6dsoCtx, PROPERTY_ENABLE);

    return 0;
}

void lsm6dso_get_acc_cali_data(uint8_t* acc_offset)
{
    if (acc_offset == NULL)
        return;

    lsm6dso_xl_usr_offset_x_get(&g_Lsm6dsoCtx, acc_offset++);
    lsm6dso_xl_usr_offset_y_get(&g_Lsm6dsoCtx, acc_offset++);
    lsm6dso_xl_usr_offset_z_get(&g_Lsm6dsoCtx, acc_offset);
}

void lsm6dso_set_acc_cali_data(uint8_t* acc_offset)
{
    if (acc_offset == NULL)
        return;

    /* Accelerometer X,Y,Z axis user offset correction expressed
    * in two's complement.
    */
    lsm6dso_xl_usr_offset_x_set(&g_Lsm6dsoCtx, acc_offset++);
    lsm6dso_xl_usr_offset_y_set(&g_Lsm6dsoCtx, acc_offset++);
    lsm6dso_xl_usr_offset_z_set(&g_Lsm6dsoCtx, acc_offset);
    lsm6dso_xl_usr_offset_set(&g_Lsm6dsoCtx, PROPERTY_ENABLE);
}

uint8_t lsm6dso_tilt_status(void)
{
    uint8_t is_tilt;

    lsm6dso_tilt_flag_data_ready_get(&g_Lsm6dsoCtx, &is_tilt);
    if (is_tilt)
        return 1;
    else
        return 0;
}
