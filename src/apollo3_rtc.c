//*****************************************************************************
//
//! @file apollo3_rtc.c
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

#include "apollo3_rtc.h"
#include "am_util.h"

//*****************************************************************************
//
// Variables used globally
//
//*****************************************************************************
am_hal_rtc_time_t hal_time;

//
// String arrays to index Days and Months with the values returned by the RTC.
//
char *pcWeekday[] =
{
    "Sunday",
    "Monday",
    "Tuesday",
    "Wednesday",
    "Thursday",
    "Friday",
    "Saturday",
    "Invalid day"
};
char *pcMonth[] =
{
    "January",
    "February",
    "March",
    "April",
    "May",
    "June",
    "July",
    "August",
    "September",
    "October",
    "November",
    "December",
    "Invalid month"
};

//*****************************************************************************
//
// Support function:
// str_to_int() converts a string to an ASCII value.
//
//*****************************************************************************
static int16_t str_to_int(char *pcAsciiStr)
{
    int16_t iRet = 0;
    int16_t i = 1;

    if (*pcAsciiStr == ' ')
        pcAsciiStr++;

    if (*pcAsciiStr == '-')
    {
        i = -1;
        pcAsciiStr++;
    }
    
    while(*pcAsciiStr >= '0' && *pcAsciiStr <= '9')
    {
        iRet *= 10;
        iRet += (*pcAsciiStr - '0') * i;
        pcAsciiStr ++;
    }
    return iRet;
}

//*****************************************************************************
//
// Support function:
// mthToIndex() converts a string indicating a month to an index value.
// The return value is a value 0-12, with 0-11 indicating the month given
// by the string, and 12 indicating that the string is not a month.
//
//*****************************************************************************
int mthToIndex(char *pcMon)
{
    int idx;
    for (idx = 0; idx < 12; idx++)
    {
        if ( am_util_string_strnicmp(pcMonth[idx], pcMon, 3) == 0 )
        {
            return idx;
        }
    }
    return 12;
}

//*****************************************************************************
//
// rtc_init
//
//*****************************************************************************
void rtc_init(void)
{
    //
    // Enable the XT for the RTC.
    //
    am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_XTAL_START, 0);

    //
    // Select XT for RTC clock source
    //
    am_hal_rtc_osc_select(AM_HAL_RTC_OSC_XT);

    //
    // Enable the RTC.
    //
    am_hal_rtc_osc_enable();

    //
    // The RTC is initialized from the date and time strings that are
    // obtained from the compiler at compile time.
    //
    hal_time.ui32Hour = str_to_int(&__TIME__[0]);
    hal_time.ui32Minute = str_to_int(&__TIME__[3]);
    hal_time.ui32Second = str_to_int(&__TIME__[6]);
    hal_time.ui32Hundredths = 00;
    hal_time.ui32Weekday = am_util_time_computeDayofWeek(2000 + str_to_int(&__DATE__[9]), mthToIndex(&__DATE__[0]) + 1, str_to_int(&__DATE__[4]) );
    hal_time.ui32DayOfMonth = str_to_int(&__DATE__[4]);
    hal_time.ui32Month = mthToIndex(&__DATE__[0]);
    hal_time.ui32Year = str_to_int(&__DATE__[9]);
    hal_time.ui32Century = 00;

    am_hal_rtc_time_set(&hal_time);

    // Set the alarm repeat interval to be every second.
    am_hal_rtc_alarm_interval_set(AM_HAL_RTC_ALM_RPT_HR);

    // Clear the RTC alarm interrupt.
    am_hal_rtc_int_clear(AM_HAL_RTC_INT_ALM);

    // Enable the RTC alarm interrupt.
    am_hal_rtc_int_enable(AM_HAL_RTC_INT_ALM);

    // Enable GPIO interrupts to the NVIC.
    NVIC_EnableIRQ(RTC_IRQn);
}

// Get the timestamp
// retval is ms
// The count will clear after one month
uint32_t get_tick_count(void)
{
    uint32_t retCountMs;

    am_hal_rtc_time_get(&hal_time);

    retCountMs = hal_time.ui32Hundredths * 10;
    retCountMs += hal_time.ui32Second * 1000;
    retCountMs += hal_time.ui32Minute * 60 * 1000;
    retCountMs += hal_time.ui32Hour * 60 * 60 * 1000;
    retCountMs += (hal_time.ui32DayOfMonth - 1) * 24 * 60 * 60 * 1000;
    
    return retCountMs;
}

//*****************************************************************************
//
// RTC ISR
//
//*****************************************************************************
void am_rtc_isr(void)
{
    //
    // Clear the RTC alarm interrupt.
    //
    am_hal_rtc_int_clear(AM_HAL_RTC_INT_ALM);

    am_hal_rtc_time_get(&hal_time); \
    am_util_stdio_printf("%02d-%02d %02d:%02d:%02d.%02d Info: Alarm", \
        hal_time.ui32Month + 1, hal_time.ui32DayOfMonth, \
        hal_time.ui32Hour, hal_time.ui32Minute, hal_time.ui32Second, \
        hal_time.ui32Hundredths); \
    am_util_stdio_printf("\n");
}
