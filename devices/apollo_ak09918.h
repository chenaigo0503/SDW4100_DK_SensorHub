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

//*****************************************************************************
//
// External function definitions
//
//*****************************************************************************
extern void ak099xx_init(void);

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
