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
#include "am_bsp.h"
#include "am_mcu_apollo.h"
#include "am_util.h"
#include "apollo_lsm6dso.h"
#include "apollo_tracelog.h"

stmdev_ctx_t g_Lsm6dsoCtx;
static uint32_t lsmBuffer[2];

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

#if (APOLLO3_HUB_VER == 1)
    am_hal_gpio_state_write(LSM6DSO_PIN_CE, AM_HAL_GPIO_OUTPUT_CLEAR);
#else
    Transaction.uPeerInfo.ui32SpiChipSelect = AM_BSP_IOM0_CS_CHNL;
#endif

    am_hal_iom_blocking_transfer(handle, &Transaction);
    memcpy(bufp, lsmBuffer, len);
#if (APOLLO3_HUB_VER == 1)
    am_hal_gpio_state_write(LSM6DSO_PIN_CE, AM_HAL_GPIO_OUTPUT_SET);
#endif
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

#if (APOLLO3_HUB_VER == 1)
    am_hal_gpio_state_write(LSM6DSO_PIN_CE, AM_HAL_GPIO_OUTPUT_CLEAR);
#else
    Transaction.uPeerInfo.ui32SpiChipSelect = AM_BSP_IOM0_CS_CHNL;
#endif

    am_hal_iom_blocking_transfer(handle, &Transaction);
#if (APOLLO3_HUB_VER == 1)
    am_hal_gpio_state_write(LSM6DSO_PIN_CE, AM_HAL_GPIO_OUTPUT_SET);
#endif
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

        ctx->read_reg(ctx, LSM6DSO_CTRL2_G, (uint8_t*)&reg, 1);
        reg.odr_g = (uint8_t) odr_gy;
        ctx->write_reg(ctx, LSM6DSO_CTRL2_G, (uint8_t*)&reg, 1);
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
         | fsm_int1_b.int1_fsm16) != PROPERTY_DISABLE){
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
    ctx->write_reg(ctx->handle, LSM6DSO_MD2_CFG, (uint8_t*)&md2_cfg, 1);
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
    lsm6dso_pin_int1_route_t int1_route;

    am_hal_iom_config_t m_sIOMSpiConfig =
    {
        .eInterfaceMode = AM_HAL_IOM_SPI_MODE,
        .ui32ClockFreq = AM_HAL_IOM_8MHZ,
        .eSpiMode = AM_HAL_IOM_SPI_MODE_0,
    };

    am_hal_iom_initialize(LSM6DSO_IOM_MODULE, &g_Lsm6dsoCtx.handle);
    am_hal_iom_power_ctrl(g_Lsm6dsoCtx.handle, AM_HAL_SYSCTRL_WAKE, false);

    // Set the required configuration settings for the IOM.
    am_hal_iom_configure(g_Lsm6dsoCtx.handle, &m_sIOMSpiConfig);

    g_Lsm6dsoCtx.read_reg = lsm6d_read;
    g_Lsm6dsoCtx.write_reg = lsm6d_write;

    // Configure the IOM pins.
#if (APOLLO3_HUB_VER == 1)
    am_hal_gpio_state_write(LSM6DSO_PIN_CE, AM_HAL_GPIO_OUTPUT_SET);
    am_hal_gpio_pinconfig(LSM6DSO_PIN_CE, g_AM_HAL_GPIO_OUTPUT_8);
#endif
    am_bsp_iom_pins_enable(LSM6DSO_IOM_MODULE, AM_HAL_IOM_SPI_MODE);

    // Enable the IOM.
    am_hal_iom_enable(g_Lsm6dsoCtx.handle);
    am_util_delay_ms(1);

    // irq delay tp config
    //am_hal_gpio_pinconfig(LSM6DSO_INTPIN_ACC, g_AM_HAL_GPIO_INPUT_PULLUP);

    /* Check device ID */
    lsm6dso_device_id_get(&g_Lsm6dsoCtx, &whoAmI);
    if(LSM6DSO_WHO_AM_I != (uint8_t)whoAmI)
        PR_ERR("ERROR: lsm6dso get ID: 0x%02x error.", whoAmI);
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

    /* Set XL and Gyro Output Data Rate */
    lsm6dso_xl_data_rate_set(&g_Lsm6dsoCtx, LSM6DSO_XL_ODR_208Hz);
    lsm6dso_gy_data_rate_set(&g_Lsm6dsoCtx, LSM6DSO_GY_ODR_104Hz);

    /* Set 2g full XL scale and 250 dps full Gyro */
    lsm6dso_xl_full_scale_set(&g_Lsm6dsoCtx, LSM6DSO_2g);
    lsm6dso_gy_full_scale_set(&g_Lsm6dsoCtx, LSM6DSO_250dps);

    /* Set duration for Activity detection to 9.62 ms (= 2 * 1 / ODR_XL) */
    lsm6dso_wkup_dur_set(&g_Lsm6dsoCtx, 0x02);

    /* Set duration for Inactivity detection to 4.92 s (= 2 * 512 / ODR_XL) */
    lsm6dso_act_sleep_dur_set(&g_Lsm6dsoCtx, 0x02);

    /* Set Activity/Inactivity threshold to 62.5 mg */
    lsm6dso_wkup_threshold_set(&g_Lsm6dsoCtx, 0x02);

    /* Inactivity configuration: XL to 12.5 in LP, gyro to Power-Down */
    lsm6dso_act_mode_set(&g_Lsm6dsoCtx, LSM6DSO_XL_12Hz5_GY_PD);

    /* Enable interrupt generation on Inactivity INT1 pin */
    lsm6dso_pin_int1_route_get(&g_Lsm6dsoCtx, &int1_route);
    int1_route.sleep_change = PROPERTY_ENABLE;
    lsm6dso_pin_int1_route_set(&g_Lsm6dsoCtx, int1_route);
    
    PR_ERR("lsm6dso init OK");
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
    ctx->read_reg(ctx, LSM6DSO_EMB_FUNC_ODR_CFG_B, (uint8_t*)&reg, 1);
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
