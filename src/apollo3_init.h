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

typedef struct {
  uint8_t lsm_irq1                 : 1;
  uint8_t lsm_irq2                 : 1;
  uint8_t akm_irq                  : 1;
  uint8_t bmp_irq                  : 1;
  uint8_t pah_irq1                 : 1;
  uint8_t pah_irq2                 : 1;
  uint8_t host_irq1                : 1;
  uint8_t host_irq2                : 1;
} apollo_irq_c_t;
extern volatile apollo_irq_c_t apollo_irq;
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
void inform_host(void);

#endif // APOLLO3_INIT_H
