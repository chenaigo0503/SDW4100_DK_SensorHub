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
    hal_time.ui32Century = 21;

    am_hal_rtc_time_set(&hal_time);
}
