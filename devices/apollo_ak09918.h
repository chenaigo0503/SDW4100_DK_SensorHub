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
#ifndef APOLLO_AK09918_H
#define APOLLO_AK09918_H

#include <stdint.h>
#include "apollo3_init.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define AK09918_IOM_MODULE 4
#define AK09918_ADDR       0x0C

// sensor ID
#define AK09918_WHO_AM_I   0xc48

// Register offset address
#define AK099XX_REG_WIA1                 0x00
#define AK099XX_REG_WIA2                 0x01
#define AK099XX_REG_INFO1                0x02
#define AK099XX_REG_INFO2                0x03
#define AK099XX_REG_ST1                  0x10
#define AK099XX_REG_HXL                  0x11
#define AK099XX_REG_HXH                  0x12
#define AK099XX_REG_HYL                  0x13
#define AK099XX_REG_HYH                  0x14
#define AK099XX_REG_HZL                  0x15
#define AK099XX_REG_HZH                  0x16
#define AK099XX_REG_TMPS                 0x17
#define AK099XX_REG_ST2                  0x18
#define AK099XX_REG_CNTL1                0x30
#define AK099XX_REG_CNTL2                0x31
#define AK099XX_REG_CNTL3                0x32

#define AK099XX_FUSE_ASAX                0x60
#define AK099XX_FUSE_ASAY                0x61
#define AK099XX_FUSE_ASAZ                0x62

#define AK099XX_BDATA_SIZE               9

#define AK099XX_MODE_SNG_MEASURE         0x01
#define AK099XX_MODE_CONT_MEASURE_MODE1  0x02
#define AK099XX_MODE_CONT_MEASURE_MODE2  0x04
#define AK099XX_MODE_CONT_MEASURE_MODE3  0x06
#define AK099XX_MODE_CONT_MEASURE_MODE4  0x08
#define AK099XX_MODE_CONT_MEASURE_MODE5  0x0A
#define AK099XX_MODE_CONT_MEASURE_MODE6  0x0C
#define AK099XX_MODE_SELF_TEST           0x10
#define AK099XX_MODE_FUSE_ACCESS         0x1F
#define AK099XX_MODE_POWER_DOWN          0x00

#define AK099XX_SOFT_RESET               0x01

#define AK09911_WIA_VAL                  0x548
#define AK09912_WIA_VAL                  0x448
#define AK09913_WIA_VAL                  0x848
#define AK09915_WIA_VAL                  0x1048
#define AK09916C_WIA_VAL                 0x948
#define AK09916D_WIA_VAL                 0xB48
#define AK09918_WIA_VAL                  0xC48
#define AK09915D_INFO_VAL                0x02

//*****************************************************************************
//
// External function definitions
//
//*****************************************************************************
extern void ak099xx_init(void);
void ak099xx_start(const int32_t freq);
void ak099xx_stop(void);
uint8_t ak099xx_check_rdy(void);
void ak099xx_get_data(int16_t data[3], int16_t st[2]);



#ifdef __cplusplus
}
#endif

#endif // APOLLO_AK09918_H

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
