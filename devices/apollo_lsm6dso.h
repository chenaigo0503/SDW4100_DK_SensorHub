//*****************************************************************************
//
//! @file apollo_lsm6dso.h
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
#ifndef APOLLO_LSM6DSO_H
#define APOLLO_LSM6DSO_H

#include <stdint.h>
#include "apollo3_init.h"

#ifdef __cplusplus
extern "C"
{
#endif

#if (APOLLO3_HUB_VER == 1)
#define LSM6DSO_PIN_CE     4
#endif
#define LSM6DSO_IOM_MODULE 0
#define LSM6DSO_PIN_INT1   14
#define LSM6DSO_PIN_INT2   15

// sensor ID
#define LSM6DSO_WHO_AM_I   0x6C

// Register offset address
#define LSM6DSO_OFFSET_ID  0x0FU

#define LSM6DSO_FUNC_CFG_ACCESS              0x01U
typedef struct {
  uint8_t not_used_01              : 6;
  uint8_t reg_access               : 2; /* shub_reg_access + func_cfg_access */
} lsm6dso_func_cfg_access_t;

#define LSM6DSO_CTRL2_G                      0x11U
typedef struct {
  uint8_t not_used_01              : 1;
  uint8_t fs_g                     : 3; /* fs_125 + fs_g */
  uint8_t odr_g                    : 4;
} lsm6dso_ctrl2_g_t;

#define LSM6DSO_CTRL3_C                      0x12U
typedef struct {
  uint8_t sw_reset                 : 1;
  uint8_t not_used_01              : 1;
  uint8_t if_inc                   : 1;
  uint8_t sim                      : 1;
  uint8_t pp_od                    : 1;
  uint8_t h_lactive                : 1;
  uint8_t bdu                      : 1;
  uint8_t boot                     : 1;
} lsm6dso_ctrl3_c_t;

#define LSM6DSO_EMB_FUNC_EN_A                0x04U
typedef struct {
  uint8_t not_used_01              : 3;
  uint8_t pedo_en                  : 1;
  uint8_t tilt_en                  : 1;
  uint8_t sign_motion_en           : 1;
  uint8_t not_used_02              : 2;
} lsm6dso_emb_func_en_a_t;

#define LSM6DSO_EMB_FUNC_EN_B                0x05U
typedef struct {
  uint8_t fsm_en                   : 1;
  uint8_t not_used_01              : 2;
  uint8_t fifo_compr_en            : 1;
  uint8_t pedo_adv_en              : 1;
  uint8_t not_used_02              : 3;
} lsm6dso_emb_func_en_b_t;

#define LSM6DSO_PAGE_ADDRESS                 0x08U
typedef struct {
  uint8_t page_addr                : 8;
} lsm6dso_page_address_t;

#define LSM6DSO_PAGE_VALUE                   0x09U
typedef struct {
  uint8_t page_value               : 8;
} lsm6dso_page_value_t;

#define PROPERTY_DISABLE                (0U)
#define PROPERTY_ENABLE                 (1U)

#define LSM6DSO_COUNTER_BDR_REG1             0x0BU
typedef struct {
  uint8_t cnt_bdr_th               : 3;
  uint8_t not_used_01              : 2;
  uint8_t trig_counter_bdr         : 1;
  uint8_t rst_counter_bdr          : 1;
  uint8_t dataready_pulsed         : 1;
} lsm6dso_counter_bdr_reg1_t;

#define LSM6DSO_EMB_FUNC_INT1                0x0AU
typedef struct {
  uint8_t not_used_01              : 3;
  uint8_t int1_step_detector       : 1;
  uint8_t int1_tilt                : 1;
  uint8_t int1_sig_mot             : 1;
  uint8_t not_used_02              : 1;
  uint8_t int1_fsm_lc              : 1;
} lsm6dso_emb_func_int1_t;

#define LSM6DSO_FSM_INT1_A                   0x0BU
typedef struct {
  uint8_t int1_fsm1                : 1;
  uint8_t int1_fsm2                : 1;
  uint8_t int1_fsm3                : 1;
  uint8_t int1_fsm4                : 1;
  uint8_t int1_fsm5                : 1;
  uint8_t int1_fsm6                : 1;
  uint8_t int1_fsm7                : 1;
  uint8_t int1_fsm8                : 1;
} lsm6dso_fsm_int1_a_t;

#define LSM6DSO_FSM_INT1_B                   0x0CU
typedef struct {
  uint8_t int1_fsm9                : 1;
  uint8_t int1_fsm10               : 1;
  uint8_t int1_fsm11               : 1;
  uint8_t int1_fsm12               : 1;
  uint8_t int1_fsm13               : 1;
  uint8_t int1_fsm14               : 1;
  uint8_t int1_fsm15               : 1;
  uint8_t int1_fsm16               : 1;
} lsm6dso_fsm_int1_b_t;

#define LSM6DSO_INT1_CTRL  0x0D
typedef struct {
  uint8_t int1_drdy_xl             : 1;
  uint8_t int1_drdy_g              : 1;
  uint8_t int1_boot                : 1;
  uint8_t int1_fifo_th             : 1;
  uint8_t int1_fifo_ovr            : 1;
  uint8_t int1_fifo_full           : 1;
  uint8_t int1_cnt_bdr             : 1;
  uint8_t den_drdy_flag            : 1;
} lsm6dso_int1_ctrl_t;

#define LSM6DSO_INT2_CTRL                    0x0EU
typedef struct {
  uint8_t int2_drdy_xl             : 1;
  uint8_t int2_drdy_g              : 1;
  uint8_t int2_drdy_temp           : 1;
  uint8_t int2_fifo_th             : 1;
  uint8_t int2_fifo_ovr            : 1;
  uint8_t int2_fifo_full           : 1;
  uint8_t int2_cnt_bdr             : 1;
  uint8_t not_used_01              : 1;
} lsm6dso_int2_ctrl_t;

#define LSM6DSO_EMB_FUNC_INT2                0x0EU
typedef struct {
  uint8_t not_used_01              : 3;
  uint8_t int2_step_detector       : 1;
  uint8_t int2_tilt                : 1;
  uint8_t int2_sig_mot             : 1;
  uint8_t not_used_02              : 1;
  uint8_t int2_fsm_lc              : 1;
} lsm6dso_emb_func_int2_t;

#define LSM6DSO_FSM_INT2_A                   0x0FU
typedef struct {
  uint8_t int2_fsm1                : 1;
  uint8_t int2_fsm2                : 1;
  uint8_t int2_fsm3                : 1;
  uint8_t int2_fsm4                : 1;
  uint8_t int2_fsm5                : 1;
  uint8_t int2_fsm6                : 1;
  uint8_t int2_fsm7                : 1;
  uint8_t int2_fsm8                : 1;
} lsm6dso_fsm_int2_a_t;

#define LSM6DSO_FSM_INT2_B                   0x10U
typedef struct {
  uint8_t int2_fsm9                : 1;
  uint8_t int2_fsm10               : 1;
  uint8_t int2_fsm11               : 1;
  uint8_t int2_fsm12               : 1;
  uint8_t int2_fsm13               : 1;
  uint8_t int2_fsm14               : 1;
  uint8_t int2_fsm15               : 1;
  uint8_t int2_fsm16               : 1;
} lsm6dso_fsm_int2_b_t;

#define LSM6DSO_CTRL1_XL                     0x10U
typedef struct {
  uint8_t not_used_01              : 1;
  uint8_t lpf2_xl_en               : 1;
  uint8_t fs_xl                    : 2;
  uint8_t odr_xl                   : 4;
} lsm6dso_ctrl1_xl_t;

#define LSM6DSO_CTRL4_C                      0x13U
typedef struct {
  uint8_t not_used_01              : 1;
  uint8_t lpf1_sel_g               : 1;
  uint8_t i2c_disable              : 1;
  uint8_t drdy_mask                : 1;
  uint8_t not_used_02              : 1;
  uint8_t int2_on_int1             : 1;
  uint8_t sleep_g                  : 1;
  uint8_t not_used_03              : 1;
} lsm6dso_ctrl4_c_t;

#define LSM6DSO_CTRL5_C                      0x14U
typedef struct {
  uint8_t st_xl                    : 2;
  uint8_t st_g                     : 2;
  uint8_t not_used_01              : 1;
  uint8_t rounding                 : 2;
  uint8_t xl_ulp_en                : 1;
} lsm6dso_ctrl5_c_t;

#define LSM6DSO_CTRL6_C                      0x15U
typedef struct {
  uint8_t ftype                    : 3;
  uint8_t usr_off_w                : 1;
  uint8_t xl_hm_mode               : 1;
  uint8_t den_mode                 : 3;   /* trig_en + lvl1_en + lvl2_en */
} lsm6dso_ctrl6_c_t;

#define LSM6DSO_CTRL7_G                      0x16U
typedef struct {
  uint8_t ois_on                   : 1;
  uint8_t usr_off_on_out           : 1;
  uint8_t ois_on_en                : 1;
  uint8_t not_used_01              : 1;
  uint8_t hpm_g                    : 2;
  uint8_t hp_en_g                  : 1;
  uint8_t g_hm_mode                : 1;
} lsm6dso_ctrl7_g_t;

#define LSM6DSO_PAGE_RW                      0x17U
typedef struct {
  uint8_t not_used_01              : 5;
  uint8_t page_rw                  : 2;  /* page_write + page_read */
  uint8_t emb_func_lir             : 1;
} lsm6dso_page_rw_t;

#define LSM6DSO_CTRL9_XL                     0x18U
typedef struct {
  uint8_t not_used_01              : 1;
  uint8_t i3c_disable              : 1;
  uint8_t den_lh                   : 1;
  uint8_t den_xl_g                 : 2;   /* den_xl_en + den_xl_g */
  uint8_t den_z                    : 1;
  uint8_t den_y                    : 1;
  uint8_t den_x                    : 1;
} lsm6dso_ctrl9_xl_t;

#define LSM6DSO_ALL_INT_SRC                  0x1AU
typedef struct {
  uint8_t ff_ia                    : 1;
  uint8_t wu_ia                    : 1;
  uint8_t single_tap               : 1;
  uint8_t double_tap               : 1;
  uint8_t d6d_ia                   : 1;
  uint8_t sleep_change_ia          : 1;
  uint8_t not_used_01              : 1;
  uint8_t timestamp_endcount       : 1;
} lsm6dso_all_int_src_t;

#define LSM6DSO_WAKE_UP_SRC                  0x1BU
typedef struct {
  uint8_t z_wu                     : 1;
  uint8_t y_wu                     : 1;
  uint8_t x_wu                     : 1;
  uint8_t wu_ia                    : 1;
  uint8_t sleep_state              : 1;
  uint8_t ff_ia                    : 1;
  uint8_t sleep_change_ia          : 1;
  uint8_t not_used_01              : 1;
} lsm6dso_wake_up_src_t;

#define LSM6DSO_TAP_SRC                      0x1CU
typedef struct {
  uint8_t z_tap                    : 1;
  uint8_t y_tap                    : 1;
  uint8_t x_tap                    : 1;
  uint8_t tap_sign                 : 1;
  uint8_t double_tap               : 1;
  uint8_t single_tap               : 1;
  uint8_t tap_ia                   : 1;
  uint8_t not_used_02              : 1;
} lsm6dso_tap_src_t;

#define LSM6DSO_D6D_SRC                      0x1DU
typedef struct {
  uint8_t xl                       : 1;
  uint8_t xh                       : 1;
  uint8_t yl                       : 1;
  uint8_t yh                       : 1;
  uint8_t zl                       : 1;
  uint8_t zh                       : 1;
  uint8_t d6d_ia                   : 1;
  uint8_t den_drdy                 : 1;
} lsm6dso_d6d_src_t;

#define LSM6DSO_STATUS_REG                   0x1EU
typedef struct {
  uint8_t xlda                     : 1;
  uint8_t gda                      : 1;
  uint8_t tda                      : 1;
  uint8_t not_used_01              : 5;
} lsm6dso_status_reg_t;

#define LSM6DSO_OUT_TEMP_L                   0x20U
#define LSM6DSO_OUT_TEMP_H                   0x21U
#define LSM6DSO_OUTX_L_G                     0x22U
#define LSM6DSO_OUTX_H_G                     0x23U
#define LSM6DSO_OUTY_L_G                     0x24U
#define LSM6DSO_OUTY_H_G                     0x25U
#define LSM6DSO_OUTZ_L_G                     0x26U
#define LSM6DSO_OUTZ_H_G                     0x27U
#define LSM6DSO_OUTX_L_A                     0x28U
#define LSM6DSO_OUTX_H_A                     0x29U
#define LSM6DSO_OUTY_L_A                     0x2AU
#define LSM6DSO_OUTY_H_A                     0x2BU
#define LSM6DSO_OUTZ_L_A                     0x2CU
#define LSM6DSO_OUTZ_H_A                     0x2DU
#define LSM6DSO_EMB_FUNC_STATUS_MAINPAGE     0x35U
typedef struct {
  uint8_t not_used_01             : 3;
  uint8_t is_step_det             : 1;
  uint8_t is_tilt                 : 1;
  uint8_t is_sigmot               : 1;
  uint8_t not_used_02             : 1;
  uint8_t is_fsm_lc               : 1;
} lsm6dso_emb_func_status_mainpage_t;

#define LSM6DSO_FSM_STATUS_A_MAINPAGE        0x36U
typedef struct {
  uint8_t is_fsm1                 : 1;
  uint8_t is_fsm2                 : 1;
  uint8_t is_fsm3                 : 1;
  uint8_t is_fsm4                 : 1;
  uint8_t is_fsm5                 : 1;
  uint8_t is_fsm6                 : 1;
  uint8_t is_fsm7                 : 1;
  uint8_t is_fsm8                 : 1;
} lsm6dso_fsm_status_a_mainpage_t;

#define LSM6DSO_FSM_STATUS_B_MAINPAGE        0x37U
typedef struct {
  uint8_t is_fsm9                 : 1;
  uint8_t is_fsm10                : 1;
  uint8_t is_fsm11                : 1;
  uint8_t is_fsm12                : 1;
  uint8_t is_fsm13                : 1;
  uint8_t is_fsm14                : 1;
  uint8_t is_fsm15                : 1;
  uint8_t is_fsm16                : 1;
} lsm6dso_fsm_status_b_mainpage_t;

#define LSM6DSO_STATUS_MASTER_MAINPAGE       0x39U
typedef struct {
  uint8_t sens_hub_endop          : 1;
  uint8_t not_used_01             : 2;
  uint8_t slave0_nack             : 1;
  uint8_t slave1_nack             : 1;
  uint8_t slave2_nack             : 1;
  uint8_t slave3_nack             : 1;
  uint8_t wr_once_done            : 1;
} lsm6dso_status_master_mainpage_t;

#define LSM6DSO_FIFO_STATUS1                 0x3AU
typedef struct {
  uint8_t diff_fifo                : 8;
} lsm6dso_fifo_status1_t;

#define LSM6DSO_FIFO_STATUS2                 0x3B
typedef struct {
  uint8_t diff_fifo                : 2;
  uint8_t not_used_01              : 1;
  uint8_t over_run_latched         : 1;
  uint8_t counter_bdr_ia           : 1;
  uint8_t fifo_full_ia             : 1;
  uint8_t fifo_ovr_ia              : 1;
  uint8_t fifo_wtm_ia              : 1;
} lsm6dso_fifo_status2_t;

#define LSM6DSO_FSM_ENABLE_A                 0x46U
typedef struct {
  uint8_t fsm1_en                  : 1;
  uint8_t fsm2_en                  : 1;
  uint8_t fsm3_en                  : 1;
  uint8_t fsm4_en                  : 1;
  uint8_t fsm5_en                  : 1;
  uint8_t fsm6_en                  : 1;
  uint8_t fsm7_en                  : 1;
  uint8_t fsm8_en                  : 1;
} lsm6dso_fsm_enable_a_t;

#define LSM6DSO_FSM_ENABLE_B                 0x47U
typedef struct {
  uint8_t fsm9_en                  : 1;
  uint8_t fsm10_en                 : 1;
  uint8_t fsm11_en                 : 1;
  uint8_t fsm12_en                 : 1;
  uint8_t fsm13_en                 : 1;
  uint8_t fsm14_en                 : 1;
  uint8_t fsm15_en                 : 1;
  uint8_t fsm16_en                 : 1;
} lsm6dso_fsm_enable_b_t;

#define LSM6DSO_TAP_CFG2                     0x58U
typedef struct {
  uint8_t tap_ths_y                : 5;
  uint8_t inact_en                 : 2;
  uint8_t interrupts_enable        : 1;
} lsm6dso_tap_cfg2_t;

#define LSM6DSO_WAKE_UP_THS                  0x5BU
typedef struct {
  uint8_t wk_ths                   : 6;
  uint8_t usr_off_on_wu            : 1;
  uint8_t single_double_tap        : 1;
} lsm6dso_wake_up_ths_t;

#define LSM6DSO_WAKE_UP_DUR                  0x5CU
typedef struct {
  uint8_t sleep_dur                : 4;
  uint8_t wake_ths_w               : 1;
  uint8_t wake_dur                 : 2;
  uint8_t ff_dur                   : 1;
} lsm6dso_wake_up_dur_t;

#define LSM6DSO_MD1_CFG                      0x5EU
typedef struct {
  uint8_t int1_shub                : 1;
  uint8_t int1_emb_func            : 1;
  uint8_t int1_6d                  : 1;
  uint8_t int1_double_tap          : 1;
  uint8_t int1_ff                  : 1;
  uint8_t int1_wu                  : 1;
  uint8_t int1_single_tap          : 1;
  uint8_t int1_sleep_change        : 1;
} lsm6dso_md1_cfg_t;

#define LSM6DSO_EMB_FUNC_ODR_CFG_B           0x5FU
typedef struct {
  uint8_t not_used_01              : 3;
  uint8_t fsm_odr                  : 2;
  uint8_t not_used_02              : 3;
} lsm6dso_emb_func_odr_cfg_b_t;

#define LSM6DSO_MD2_CFG                      0x5FU
typedef struct {
  uint8_t int2_timestamp           : 1;
  uint8_t int2_emb_func            : 1;
  uint8_t int2_6d                  : 1;
  uint8_t int2_double_tap          : 1;
  uint8_t int2_ff                  : 1;
  uint8_t int2_wu                  : 1;
  uint8_t int2_single_tap          : 1;
  uint8_t int2_sleep_change        : 1;
} lsm6dso_md2_cfg_t;

#define LSM6DSO_I3C_BUS_AVB                  0x62U
typedef struct {
  uint8_t pd_dis_int1              : 1;
  uint8_t not_used_01              : 2;
  uint8_t i3c_bus_avb_sel          : 2;
  uint8_t not_used_02              : 3;
} lsm6dso_i3c_bus_avb_t;

#define LSM6DSO_STEP_COUNTER_L               0x62U
#define LSM6DSO_STEP_COUNTER_H               0x63U
#define LSM6DSO_EMB_FUNC_SRC                 0x64U
typedef struct {
  uint8_t not_used_01              : 2;
  uint8_t stepcounter_bit_set      : 1;
  uint8_t step_overflow            : 1;
  uint8_t step_count_delta_ia      : 1;
  uint8_t step_detected            : 1;
  uint8_t not_used_02              : 1;
  uint8_t pedo_rst_step            : 1;
} lsm6dso_emb_func_src_t;

#define LSM6DSO_INT_OIS                      0x6FU
typedef struct {
  uint8_t st_xl_ois                : 2;
  uint8_t not_used_01              : 3;
  uint8_t den_lh_ois               : 1;
  uint8_t lvl2_ois                 : 1;
  uint8_t int2_drdy_ois            : 1;
} lsm6dso_int_ois_t;

#define LSM6DSO_X_OFS_USR                    0x73U
#define LSM6DSO_Y_OFS_USR                    0x74U
#define LSM6DSO_Z_OFS_USR                    0x75U
#define LSM6DSO_FIFO_DATA_OUT_TAG            0x78U
typedef struct {
  uint8_t tag_parity               : 1;
  uint8_t tag_cnt                  : 2;
  uint8_t tag_sensor               : 5;
} lsm6dso_fifo_data_out_tag_t;

#define LSM6DSO_FIFO_DATA_OUT_X_L            0x79U
#define LSM6DSO_FIFO_DATA_OUT_X_H            0x7AU
#define LSM6DSO_FIFO_DATA_OUT_Y_L            0x7BU
#define LSM6DSO_FIFO_DATA_OUT_Y_H            0x7CU
#define LSM6DSO_FIFO_DATA_OUT_Z_L            0x7DU
#define LSM6DSO_FIFO_DATA_OUT_Z_H            0x7EU
#define LSM6DSO_PAGE_SEL                     0x02U
typedef struct {
  uint8_t not_used_01              : 4;
  uint8_t page_sel                 : 4;
} lsm6dso_page_sel_t;

#define LSM6DSO_FSM_LC_TIMEOUT_L             0x17AU
#define LSM6DSO_FSM_LC_TIMEOUT_H             0x17BU
#define LSM6DSO_FSM_PROGRAMS                 0x17CU
#define LSM6DSO_FSM_START_ADD_L              0x17EU
#define LSM6DSO_FSM_START_ADD_H              0x17FU
#define LSM6DSO_PEDO_CMD_REG                 0x183U
typedef struct {
  uint8_t ad_det_en                : 1;
  uint8_t not_used_01              : 1;
  uint8_t fp_rejection_en          : 1;
  uint8_t carry_count_en           : 1;
  uint8_t not_used_02              : 4;
} lsm6dso_pedo_cmd_reg_t;

typedef void (*stmdev_write_ptr) (void *, uint8_t, uint8_t*, uint32_t);
typedef void (*stmdev_read_ptr) (void *, uint8_t, uint8_t*, uint32_t);

typedef struct {
  /** Component mandatory fields **/
  stmdev_write_ptr  write_reg;
  stmdev_read_ptr   read_reg;
  /** Customizable optional pointer **/
  void *handle;
} stmdev_ctx_t;

typedef enum {
  LSM6DSO_I3C_DISABLE         = 0x80,
  LSM6DSO_I3C_ENABLE_T_50us   = 0x00,
  LSM6DSO_I3C_ENABLE_T_2us    = 0x01,
  LSM6DSO_I3C_ENABLE_T_1ms    = 0x02,
  LSM6DSO_I3C_ENABLE_T_25ms   = 0x03,
} lsm6dso_i3c_disable_t;

typedef enum {
  LSM6DSO_XL_ODR_OFF    = 0,
  LSM6DSO_XL_ODR_12Hz5  = 1,
  LSM6DSO_XL_ODR_26Hz   = 2,
  LSM6DSO_XL_ODR_52Hz   = 3,
  LSM6DSO_XL_ODR_104Hz  = 4,
  LSM6DSO_XL_ODR_208Hz  = 5,
  LSM6DSO_XL_ODR_417Hz  = 6,
  LSM6DSO_XL_ODR_833Hz  = 7,
  LSM6DSO_XL_ODR_1667Hz = 8,
  LSM6DSO_XL_ODR_3333Hz = 9,
  LSM6DSO_XL_ODR_6667Hz = 10,
  LSM6DSO_XL_ODR_1Hz6   = 11, /* (low power only) */
} lsm6dso_odr_xl_t;

typedef struct {
    lsm6dso_fsm_enable_a_t          fsm_enable_a;
    lsm6dso_fsm_enable_b_t          fsm_enable_b;
} lsm6dso_emb_fsm_enable_t;

typedef enum {
  LSM6DSO_ODR_FSM_12Hz5 = 0,
  LSM6DSO_ODR_FSM_26Hz  = 1,
  LSM6DSO_ODR_FSM_52Hz  = 2,
  LSM6DSO_ODR_FSM_104Hz = 3,
} lsm6dso_fsm_odr_t;

typedef enum {
  LSM6DSO_USER_BANK           = 0,
  LSM6DSO_SENSOR_HUB_BANK     = 1,
  LSM6DSO_EMBEDDED_FUNC_BANK  = 2,
} lsm6dso_reg_access_t;

typedef enum {
  LSM6DSO_GY_ODR_OFF    = 0,
  LSM6DSO_GY_ODR_12Hz5  = 1,
  LSM6DSO_GY_ODR_26Hz   = 2,
  LSM6DSO_GY_ODR_52Hz   = 3,
  LSM6DSO_GY_ODR_104Hz  = 4,
  LSM6DSO_GY_ODR_208Hz  = 5,
  LSM6DSO_GY_ODR_417Hz  = 6,
  LSM6DSO_GY_ODR_833Hz  = 7,
  LSM6DSO_GY_ODR_1667Hz = 8,
  LSM6DSO_GY_ODR_3333Hz = 9,
  LSM6DSO_GY_ODR_6667Hz = 10,
} lsm6dso_odr_g_t;

typedef enum {
  LSM6DSO_2g   = 0,
  LSM6DSO_16g  = 1, /* if XL_FS_MODE = '1' -> LSM6DSO_2g */
  LSM6DSO_4g   = 2,
  LSM6DSO_8g   = 3,
} lsm6dso_fs_xl_t;

typedef enum {
  LSM6DSO_250dps   = 0,
  LSM6DSO_125dps   = 1,
  LSM6DSO_500dps   = 2,
  LSM6DSO_1000dps  = 4,
  LSM6DSO_2000dps  = 6,
} lsm6dso_fs_g_t;

typedef enum {
  LSM6DSO_XL_AND_GY_NOT_AFFECTED      = 0,
  LSM6DSO_XL_12Hz5_GY_NOT_AFFECTED    = 1,
  LSM6DSO_XL_12Hz5_GY_SLEEP           = 2,
  LSM6DSO_XL_12Hz5_GY_PD              = 3,
} lsm6dso_inact_en_t;

typedef struct {
  uint8_t drdy_xl       : 1; /* Accelerometer data ready */
  uint8_t drdy_g        : 1; /* Gyroscope data ready */
  uint8_t drdy_temp     : 1; /* Temperature data ready (1 = int2 pin disable) */
  uint8_t boot          : 1; /* Restoring calibration parameters */
  uint8_t fifo_th       : 1; /* FIFO threshold reached */
  uint8_t fifo_ovr      : 1; /* FIFO overrun */
  uint8_t fifo_full     : 1; /* FIFO full */
  uint8_t fifo_bdr      : 1; /* FIFO Batch counter threshold reached */
  uint8_t den_flag      : 1; /* external trigger level recognition (DEN) */
  uint8_t sh_endop      : 1; /* sensor hub end operation */
  uint8_t timestamp     : 1; /* timestamp overflow (1 = int2 pin disable) */
  uint8_t six_d         : 1; /* orientation change (6D/4D detection) */
  uint8_t double_tap    : 1; /* double-tap event */
  uint8_t free_fall     : 1; /* free fall event */
  uint8_t wake_up       : 1; /* wake up event */
  uint8_t single_tap    : 1; /* single-tap event */
  uint8_t sleep_change  : 1; /* Act/Inact (or Vice-versa) status changed */
  uint8_t step_detector : 1; /* Step detected */
  uint8_t tilt          : 1; /* Relative tilt event detected */
  uint8_t sig_mot       : 1; /* "significant motion" event detected */
  uint8_t fsm_lc        : 1; /* fsm long counter timeout interrupt event */
  uint8_t fsm1          : 1; /* fsm 1 interrupt event */
  uint8_t fsm2          : 1; /* fsm 2 interrupt event */
  uint8_t fsm3          : 1; /* fsm 3 interrupt event */
  uint8_t fsm4          : 1; /* fsm 4 interrupt event */
  uint8_t fsm5          : 1; /* fsm 5 interrupt event */
  uint8_t fsm6          : 1; /* fsm 6 interrupt event */
  uint8_t fsm7          : 1; /* fsm 7 interrupt event */
  uint8_t fsm8          : 1; /* fsm 8 interrupt event */
  uint8_t fsm9          : 1; /* fsm 9 interrupt event */
  uint8_t fsm10         : 1; /* fsm 10 interrupt event */
  uint8_t fsm11         : 1; /* fsm 11 interrupt event */
  uint8_t fsm12         : 1; /* fsm 12 interrupt event */
  uint8_t fsm13         : 1; /* fsm 13 interrupt event */
  uint8_t fsm14         : 1; /* fsm 14 interrupt event */
  uint8_t fsm15         : 1; /* fsm 15 interrupt event */
  uint8_t fsm16         : 1; /* fsm 16 interrupt event */
  uint8_t mlc1          : 1; /* mlc 1 interrupt event */
  uint8_t mlc2          : 1; /* mlc 2 interrupt event */
  uint8_t mlc3          : 1; /* mlc 3 interrupt event */
  uint8_t mlc4          : 1; /* mlc 4 interrupt event */
  uint8_t mlc5          : 1; /* mlc 5 interrupt event */
  uint8_t mlc6          : 1; /* mlc 6 interrupt event */
  uint8_t mlc7          : 1; /* mlc 7 interrupt event */
  uint8_t mlc8          : 1; /* mlc 8 interrupt event */
} lsm6dso_pin_int1_route_t;
typedef enum {
  LSM6DSO_int1_drdy_xl,
  LSM6DSO_int1_drdy_g,
  LSM6DSO_int1_drdy_temp,
  LSM6DSO_int1_boot,
  LSM6DSO_int1_fifo_th,
  LSM6DSO_int1_fifo_ovr,
  LSM6DSO_int1_fifo_full,
  LSM6DSO_int1_fifo_bdr,
  LSM6DSO_int1_den_flag,
  LSM6DSO_int1_sh_endop,
  LSM6DSO_int1_timestamp,
  LSM6DSO_int1_six_d,
  LSM6DSO_int1_double_tap,
  LSM6DSO_int1_free_fall,
  LSM6DSO_int1_wake_up,
  LSM6DSO_int1_single_tap,
  LSM6DSO_int1_sleep_change,
  LSM6DSO_int1_step_detector,
  LSM6DSO_int1_tilt,
  LSM6DSO_int1_sig_mot,
  LSM6DSO_int1_fsm_lc,
  LSM6DSO_int1_fsm1,
  LSM6DSO_int1_fsm2,
  LSM6DSO_int1_fsm3,
  LSM6DSO_int1_fsm4,
  LSM6DSO_int1_fsm5,
  LSM6DSO_int1_fsm6,
  LSM6DSO_int1_fsm7,
  LSM6DSO_int1_fsm8,
  LSM6DSO_int1_fsm9,
  LSM6DSO_int1_fsm10,
  LSM6DSO_int1_fsm11,
  LSM6DSO_int1_fsm12,
  LSM6DSO_int1_fsm13,
  LSM6DSO_int1_fsm14,
  LSM6DSO_int1_fsm15,
  LSM6DSO_int1_fsm16,
  LSM6DSO_int1_mlc1,
  LSM6DSO_int1_mlc2,
  LSM6DSO_int1_mlc3,
  LSM6DSO_int1_mlc4,
  LSM6DSO_int1_mlc5,
  LSM6DSO_int1_mlc6,
  LSM6DSO_int1_mlc7,
  LSM6DSO_int1_mlc8,
  LSM6DSO_int1_all,
} lsm6dso_int1_type_t;

typedef struct {
  uint8_t drdy_ois      : 1; /* OIS chain data ready */
  uint8_t drdy_xl       : 1; /* Accelerometer data ready */
  uint8_t drdy_g        : 1; /* Gyroscope data ready */
  uint8_t drdy_temp     : 1; /* Temperature data ready */
  uint8_t fifo_th       : 1; /* FIFO threshold reached */
  uint8_t fifo_ovr      : 1; /* FIFO overrun */
  uint8_t fifo_full     : 1; /* FIFO full */
  uint8_t fifo_bdr      : 1; /* FIFO Batch counter threshold reached */
  uint8_t timestamp     : 1; /* timestamp overflow */
  uint8_t six_d         : 1; /* orientation change (6D/4D detection) */
  uint8_t double_tap    : 1; /* double-tap event */
  uint8_t free_fall     : 1; /* free fall event */
  uint8_t wake_up       : 1; /* wake up event */
  uint8_t single_tap    : 1; /* single-tap event */
  uint8_t sleep_change  : 1; /* Act/Inact (or Vice-versa) status changed */
  uint8_t step_detector : 1; /* Step detected */
  uint8_t tilt          : 1; /* Relative tilt event detected */
  uint8_t sig_mot       : 1; /* "significant motion" event detected */
  uint8_t fsm_lc        : 1; /* fsm long counter timeout interrupt event */
  uint8_t fsm1          : 1; /* fsm 1 interrupt event */
  uint8_t fsm2          : 1; /* fsm 2 interrupt event */
  uint8_t fsm3          : 1; /* fsm 3 interrupt event */
  uint8_t fsm4          : 1; /* fsm 4 interrupt event */
  uint8_t fsm5          : 1; /* fsm 5 interrupt event */
  uint8_t fsm6          : 1; /* fsm 6 interrupt event */
  uint8_t fsm7          : 1; /* fsm 7 interrupt event */
  uint8_t fsm8          : 1; /* fsm 8 interrupt event */
  uint8_t fsm9          : 1; /* fsm 9 interrupt event */
  uint8_t fsm10         : 1; /* fsm 10 interrupt event */
  uint8_t fsm11         : 1; /* fsm 11 interrupt event */
  uint8_t fsm12         : 1; /* fsm 12 interrupt event */
  uint8_t fsm13         : 1; /* fsm 13 interrupt event */
  uint8_t fsm14         : 1; /* fsm 14 interrupt event */
  uint8_t fsm15         : 1; /* fsm 15 interrupt event */
  uint8_t fsm16         : 1; /* fsm 16 interrupt event */
  uint8_t mlc1          : 1; /* mlc 1 interrupt event */
  uint8_t mlc2          : 1; /* mlc 2 interrupt event */
  uint8_t mlc3          : 1; /* mlc 3 interrupt event */
  uint8_t mlc4          : 1; /* mlc 4 interrupt event */
  uint8_t mlc5          : 1; /* mlc 5 interrupt event */
  uint8_t mlc6          : 1; /* mlc 6 interrupt event */
  uint8_t mlc7          : 1; /* mlc 7 interrupt event */
  uint8_t mlc8          : 1; /* mlc 8 interrupt event */
} lsm6dso_pin_int2_route_t;
typedef enum {
  LSM6DSO_int2_drdy_ois,
  LSM6DSO_int2_drdy_xl,
  LSM6DSO_int2_drdy_g,
  LSM6DSO_int2_drdy_temp,
  LSM6DSO_int2_fifo_th,
  LSM6DSO_int2_fifo_ovr,
  LSM6DSO_int2_fifo_full,
  LSM6DSO_int2_fifo_bdr,
  LSM6DSO_int2_timestamp,
  LSM6DSO_int2_six_d,
  LSM6DSO_int2_double_tap,
  LSM6DSO_int2_free_fall,
  LSM6DSO_int2_wake_up,
  LSM6DSO_int2_single_tap,
  LSM6DSO_int2_sleep_change,
  LSM6DSO_int2_step_detector,
  LSM6DSO_int2_tilt,
  LSM6DSO_int2_sig_mot,
  LSM6DSO_int2_fsm_lc,
  LSM6DSO_int2_fsm1,
  LSM6DSO_int2_fsm2,
  LSM6DSO_int2_fsm3,
  LSM6DSO_int2_fsm4,
  LSM6DSO_int2_fsm5,
  LSM6DSO_int2_fsm6,
  LSM6DSO_int2_fsm7,
  LSM6DSO_int2_fsm8,
  LSM6DSO_int2_fsm9,
  LSM6DSO_int2_fsm10,
  LSM6DSO_int2_fsm11,
  LSM6DSO_int2_fsm12,
  LSM6DSO_int2_fsm13,
  LSM6DSO_int2_fsm14,
  LSM6DSO_int2_fsm15,
  LSM6DSO_int2_fsm16,
  LSM6DSO_int2_mlc1,
  LSM6DSO_int2_mlc2,
  LSM6DSO_int2_mlc3,
  LSM6DSO_int2_mlc4,
  LSM6DSO_int2_mlc5,
  LSM6DSO_int2_mlc6,
  LSM6DSO_int2_mlc7,
  LSM6DSO_int2_mlc8,
  LSM6DSO_int2_all,
} lsm6dso_int2_type_t;

typedef struct {
  uint8_t drdy_xl          :  1; /* Accelerometer data ready */
  uint8_t drdy_g           :  1; /* Gyroscope data ready */
  uint8_t drdy_temp        :  1; /* Temperature data ready */
  uint8_t den_flag         :  1; /* external trigger level recognition (DEN) */
  uint8_t timestamp        :  1; /* timestamp overflow (1 = int2 pin disable) */
  uint8_t free_fall        :  1; /* free fall event */
  uint8_t wake_up          :  1; /* wake up event */
  uint8_t wake_up_z        :  1; /* wake up on Z axis event */
  uint8_t wake_up_y        :  1; /* wake up on Y axis event */
  uint8_t wake_up_x        :  1; /* wake up on X axis event */
  uint8_t single_tap       :  1; /* single-tap event */
  uint8_t double_tap       :  1; /* double-tap event */
  uint8_t tap_z            :  1; /* single-tap on Z axis event */
  uint8_t tap_y            :  1; /* single-tap on Y axis event */
  uint8_t tap_x            :  1; /* single-tap on X axis event */
  uint8_t tap_sign         :  1; /* sign of tap event (0-pos / 1-neg) */
  uint8_t six_d            :  1; /* orientation change (6D/4D detection) */
  uint8_t six_d_xl         :  1; /* X-axis low 6D/4D event (under threshold) */
  uint8_t six_d_xh         :  1; /* X-axis high 6D/4D event (over threshold) */
  uint8_t six_d_yl         :  1; /* Y-axis low 6D/4D event (under threshold) */
  uint8_t six_d_yh         :  1; /* Y-axis high 6D/4D event (over threshold) */
  uint8_t six_d_zl         :  1; /* Z-axis low 6D/4D event (under threshold) */
  uint8_t six_d_zh         :  1; /* Z-axis high 6D/4D event (over threshold) */
  uint8_t sleep_change     :  1; /* Act/Inact (or Vice-versa) status changed */
  uint8_t sleep_state      :  1; /* Act/Inact status flag (0-Act / 1-Inact) */
  uint8_t step_detector    :  1; /* Step detected */
  uint8_t tilt             :  1; /* Relative tilt event detected */
  uint8_t sig_mot          :  1; /* "significant motion" event detected */
  uint8_t fsm_lc           :  1; /* fsm long counter timeout interrupt event */
  uint8_t fsm1             :  1; /* fsm 1 interrupt event */
  uint8_t fsm2             :  1; /* fsm 2 interrupt event */
  uint8_t fsm3             :  1; /* fsm 3 interrupt event */
  uint8_t fsm4             :  1; /* fsm 4 interrupt event */
  uint8_t fsm5             :  1; /* fsm 5 interrupt event */
  uint8_t fsm6             :  1; /* fsm 6 interrupt event */
  uint8_t fsm7             :  1; /* fsm 7 interrupt event */
  uint8_t fsm8             :  1; /* fsm 8 interrupt event */
  uint8_t fsm9             :  1; /* fsm 9 interrupt event */
  uint8_t fsm10            :  1; /* fsm 10 interrupt event */
  uint8_t fsm11            :  1; /* fsm 11 interrupt event */
  uint8_t fsm12            :  1; /* fsm 12 interrupt event */
  uint8_t fsm13            :  1; /* fsm 13 interrupt event */
  uint8_t fsm14            :  1; /* fsm 14 interrupt event */
  uint8_t fsm15            :  1; /* fsm 15 interrupt event */
  uint8_t fsm16            :  1; /* fsm 16 interrupt event */
  uint8_t mlc1             :  1; /* mlc 1 interrupt event */
  uint8_t mlc2             :  1; /* mlc 2 interrupt event */
  uint8_t mlc3             :  1; /* mlc 3 interrupt event */
  uint8_t mlc4             :  1; /* mlc 4 interrupt event */
  uint8_t mlc5             :  1; /* mlc 5 interrupt event */
  uint8_t mlc6             :  1; /* mlc 6 interrupt event */
  uint8_t mlc7             :  1; /* mlc 7 interrupt event */
  uint8_t mlc8             :  1; /* mlc 8 interrupt event */
  uint8_t sh_endop         :  1; /* sensor hub end operation */
  uint8_t sh_slave0_nack   :  1; /* Not acknowledge on sensor hub slave 0 */
  uint8_t sh_slave1_nack   :  1; /* Not acknowledge on sensor hub slave 1 */
  uint8_t sh_slave2_nack   :  1; /* Not acknowledge on sensor hub slave 2 */
  uint8_t sh_slave3_nack   :  1; /* Not acknowledge on sensor hub slave 3 */
  uint8_t sh_wr_once       :  1; /* "WRITE_ONCE" end on sensor hub slave 0 */
  uint16_t fifo_diff       : 10; /* Number of unread sensor data in FIFO*/
  uint8_t fifo_ovr_latched :  1; /* Latched FIFO overrun status */
  uint8_t fifo_bdr         :  1; /* FIFO Batch counter threshold reached */
  uint8_t fifo_full        :  1; /* FIFO full */
  uint8_t fifo_ovr         :  1; /* FIFO overrun */
  uint8_t fifo_th          :  1; /* FIFO threshold reached */
} lsm6dso_all_sources_t;

typedef enum {
  LSM6DSO_DRDY_LATCHED = 0,
  LSM6DSO_DRDY_PULSED  = 1,
} lsm6dso_dataready_pulsed_t;

typedef enum {
  LSM6DSO_PEDO_DISABLE              = 0x00,
  LSM6DSO_PEDO_BASE_MODE            = 0x01,
  LSM6DSO_FALSE_STEP_REJ            = 0x13,
  LSM6DSO_FALSE_STEP_REJ_ADV_MODE   = 0x33,
} lsm6dso_pedo_md_t;

typedef enum {
  LSM6DSO_LSb_1mg  = 0,
  LSM6DSO_LSb_16mg = 1,
} lsm6dso_usr_off_w_t;

//*****************************************************************************
//
// External function definitions
//
//*****************************************************************************
extern void lsm6dso_init(void);
void lsm6dso_fsm_enable_get(stmdev_ctx_t *ctx,
                               lsm6dso_emb_fsm_enable_t *val);
void lsm6dso_fsm_data_rate_get(stmdev_ctx_t *ctx, lsm6dso_fsm_odr_t *val);
void lsm6dso_mem_bank_set(stmdev_ctx_t *ctx, lsm6dso_reg_access_t val);
void lsm6dso_int1_route_set(lsm6dso_int1_type_t int1_type, bool int1_val);
void lsm6dso_int2_route_set(lsm6dso_int2_type_t int2_type, bool int2_val);
uint16_t lsm6dso_step_get(void);

// The function that needs to be put into the loop task call
uint8_t lsm6dso_acceleration_get(float* acc_data);
uint8_t lsm6dso_angular_get(float* gyro_data);
uint8_t lsm6dso_acc_cali(void);
void lsm6dso_get_acc_cali_data(uint8_t* acc_offset);
void lsm6dso_set_acc_cali_data(uint8_t* acc_offset);

#ifdef __cplusplus
}
#endif

#endif // APOLLO_LSM6DSO_H

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
