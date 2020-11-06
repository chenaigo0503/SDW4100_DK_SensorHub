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

#if (APOLLO_LOG_ERROR < APOLLO_LOG_LEVEL)
#define PR_ERR(...) am_util_stdio_printf(__VA_ARGS__)
#else
#define PR_ERR(...)
#endif

#if (APOLLO_LOG_DEBUG < APOLLO_LOG_LEVEL)
#define PR_DBG(...) am_util_stdio_printf(__VA_ARGS__)
#else
#define PR_DBG(...)
#endif

#if (APOLLO_LOG_INFO < APOLLO_LOG_LEVEL)
#define PR_INFO(...) am_util_stdio_printf(__VA_ARGS__)
#else
#define PR_INFO(...)
#endif



#endif // APOLLO_TRACELOG_H
