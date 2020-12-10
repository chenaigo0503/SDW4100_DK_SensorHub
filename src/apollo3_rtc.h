//*****************************************************************************
//
//! @file apollo3_rtc.h
//!
//! @brief Real Time Clock use with apollo3
//
//*****************************************************************************

//*****************************************************************************
//
// Copyright (c) 2020, Thundercomm
// All rights reserved.
//
//*****************************************************************************
#ifndef APOLLO3_RTC_H
#define APOLLO3_RTC_H

#include "am_mcu_apollo.h"

extern am_hal_rtc_time_t hal_time;
extern char *pcWeekday[];
extern char *pcMonth[];

//*****************************************************************************
//
// Variables used globally
//
//*****************************************************************************
void rtc_init(void);
uint32_t get_tick_count(void);


#endif // APOLLO3_RTC_H
