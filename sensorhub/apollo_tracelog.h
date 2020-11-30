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

#define APOLLO_LOG_LEVEL 3

#define APOLLO_LOG_ERROR 0
#define APOLLO_LOG_DEBUG 1
#define APOLLO_LOG_INFO  2

#define END_LINE am_util_stdio_printf("\n\r")

#if (APOLLO_LOG_ERROR < APOLLO_LOG_LEVEL)
#define PR_ERR(...) do{am_util_stdio_printf(__VA_ARGS__);END_LINE;}while(0)
#else
#define PR_ERR(...)
#endif

#if (APOLLO_LOG_DEBUG < APOLLO_LOG_LEVEL)
#define PR_DBG(...) do{am_util_stdio_printf(__VA_ARGS__);END_LINE;}while(0)
#else
#define PR_DBG(...)
#endif

#if (APOLLO_LOG_INFO < APOLLO_LOG_LEVEL)
#define PR_INFO(...) do{am_util_stdio_printf(__VA_ARGS__);END_LINE;}while(0)
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
#define PR_info(...) am_util_stdio_printf(__VA_ARGS__)
#else
#define PR_info(...)
#endif


#endif // APOLLO_TRACELOG_H
