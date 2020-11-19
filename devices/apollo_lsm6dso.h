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
#define LSM6DSO_INTPIN_ACC 14
#define LSM6DSO_INTPIN_GYR 15

// sensor ID
#define LSM6DSO_WHO_AM_I   0x6C

// Register offset address
#define LSM6DSO_OFFSET_ID  0x0FU

#define LSM6DSO_CTRL3_C    0x12U
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

#define PROPERTY_DISABLE                (0U)
#define PROPERTY_ENABLE                 (1U)


typedef void (*stmdev_write_ptr) (void *, uint8_t, uint32_t*, uint32_t);
typedef void (*stmdev_read_ptr) (void *, uint8_t, uint32_t*, uint32_t);

typedef struct {
  /** Component mandatory fields **/
  stmdev_write_ptr  write_reg;
  stmdev_read_ptr   read_reg;
  /** Customizable optional pointer **/
  void *handle;
} stmdev_ctx_t;
//*****************************************************************************
//
// External function definitions
//
//*****************************************************************************
extern void lsm6dso_init(void);

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
