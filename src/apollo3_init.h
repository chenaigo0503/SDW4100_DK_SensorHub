//*****************************************************************************
//
//! @file apollo3_init.c
//!
//! @brief A few init functions for use with apollo3
//
//*****************************************************************************

//*****************************************************************************
//
// Copyright (c) 2020, Thundercomm
// All rights reserved.
//
//*****************************************************************************
#ifndef APOLLO3_INIT_H
#define APOLLO3_INIT_H

#include "am_mcu_apollo.h"
//*****************************************************************************
//
// Variables used globally
//
//*****************************************************************************
extern void* g_IOMArray[6];

// HW VER v1 Not use SWO
#define APOLLO3_HUB_VER 1

// apollo3_init
void apollo3_init(void);

#endif // APOLLO3_INIT_H
