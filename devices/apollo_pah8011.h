//*****************************************************************************
//
//! @file apollo_pah8011.h
//!
//! @brief Functions for controlling pah8011(Hert Rate)
//!
//! @addtogroup devices External Device Control Library
//! @addtogroup IIC Device Control for pah8011.
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
#ifndef APOLLO_PAH8011_H
#define APOLLO_PAH8011_H

#include <stdint.h>
#include "apollo3_init.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define PAH8011_IOM_MODULE 3
#define PAH8011_ADDR       0x15

// sensor ID
#define PAH8011_WHO_AM_I   0x11

// Register offset address
#define PAH8011_SETBANK_ADDR  0x7F

// Register Bank0
#define PAH8011_DEVICEID_ADDR 0x00

// Register Bank4
#define PAH8011_PD_MODE_ENH   0x69

//*****************************************************************************
//
// External function definitions
//
//*****************************************************************************
extern void pah8011_init(void);

#ifdef __cplusplus
}
#endif

#endif // APOLLO_PAH8011_H

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
