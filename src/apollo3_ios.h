//*****************************************************************************
//
//! @file apollo3_ios.c
//!
//! @brief The use of the IOS FIFO.
//
//*****************************************************************************

//*****************************************************************************
//
// Copyright (c) 2020, Thundercomm
// All rights reserved.
//
//*****************************************************************************
#ifndef APOLLO3_IOS_H
#define APOLLO3_IOS_H

#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"

// Reserve line changes to use SPI, otherwise use I2C
#define APOLLO3_IOS_USE_SPI
#ifndef APOLLO3_IOS_USE_SPI
#define APOLLO3_IOS_USE_I2C
#define I2C_ADDR              0x10
#endif

#define AM_HAL_IOS_INT_ERR    (AM_HAL_IOS_INT_FOVFL | AM_HAL_IOS_INT_FUNDFL | AM_HAL_IOS_INT_FRDERR)
#define AM_HAL_IOS_XCMP_INT   (AM_HAL_IOS_INT_XCMPWR | AM_HAL_IOS_INT_XCMPWF | AM_HAL_IOS_INT_XCMPRR | AM_HAL_IOS_INT_XCMPRF)

#define APOLLO3_IOSINT_PIN    28
#define APOLLO3_IOS_TXBUF_MAX 1023

//*****************************************************************************
//
// IOS handle.
//
//*****************************************************************************



#endif // APOLLO3_IOS_H
