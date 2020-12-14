//*****************************************************************************
//
//! @file apollo_tracelog.h
//!
//! @brief This file provides the trace log function and define log level.
//!
//
//*****************************************************************************
#ifndef APOLLO_TRACELOG_H
#define APOLLO_TRACELOG_H

#include "am_util_stdio.h"
#include "apollo3_rtc.h"

#define APOLLO_LOG_LEVEL 3

#define APOLLO_LOG_ERROR 0
#define APOLLO_LOG_DEBUG 1
#define APOLLO_LOG_INFO  2

#define END_LINE am_util_stdio_printf("\n\r")

#if (APOLLO_LOG_ERROR < APOLLO_LOG_LEVEL)
#define PR_ERR(...) do{ \
    am_hal_rtc_time_get(&hal_time); \
    am_util_stdio_printf("%02d-%02d %02d:%02d:%02d.%02d Error: ", \
        hal_time.ui32Month + 1, hal_time.ui32DayOfMonth, \
        hal_time.ui32Hour, hal_time.ui32Minute, hal_time.ui32Second, \
        hal_time.ui32Hundredths); \
    am_util_stdio_printf(__VA_ARGS__); \
    END_LINE; \
    }while(0)
#else
#define PR_ERR(...)
#endif

#if (APOLLO_LOG_DEBUG < APOLLO_LOG_LEVEL)
#define PR_DBG(...) do{ \
    am_hal_rtc_time_get(&hal_time); \
    am_util_stdio_printf("%02d-%02d %02d:%02d:%02d.%02d Debug: ", \
        hal_time.ui32Month + 1, hal_time.ui32DayOfMonth, \
        hal_time.ui32Hour, hal_time.ui32Minute, hal_time.ui32Second, \
        hal_time.ui32Hundredths); \
    am_util_stdio_printf(__VA_ARGS__); \
    END_LINE; \
    }while(0)
#else
#define PR_DBG(...)
#endif

#if (APOLLO_LOG_INFO < APOLLO_LOG_LEVEL)
#define PR_INFO(...) do{ \
    am_hal_rtc_time_get(&hal_time); \
    am_util_stdio_printf("%02d-%02d %02d:%02d:%02d.%02d Info: ", \
        hal_time.ui32Month + 1, hal_time.ui32DayOfMonth, \
        hal_time.ui32Hour, hal_time.ui32Minute, hal_time.ui32Second, \
        hal_time.ui32Hundredths); \
    am_util_stdio_printf(__VA_ARGS__); \
    END_LINE; \
    }while(0)
#else
#define PR_INFO(...)
#endif

#if (APOLLO_LOG_ERROR < APOLLO_LOG_LEVEL)
#define pr_err(...) am_util_stdio_printf(__VA_ARGS__)
#else
#define pr_err(...)
#endif

#if (APOLLO_LOG_DEBUG < APOLLO_LOG_LEVEL)
#define pr_dbg(...) am_util_stdio_printf(__VA_ARGS__)
#else
#define pr_dbg(...)
#endif

#if (APOLLO_LOG_INFO < APOLLO_LOG_LEVEL)
#define pr_info(...) am_util_stdio_printf(__VA_ARGS__)
#else
#define PR_info(...)
#endif


#endif // APOLLO_TRACELOG_H
