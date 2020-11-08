//*****************************************************************************
//
//! @file apollo_delay2run.h
//!
//! @brief Use a timer to suspend a task before it executes.
//!
//! @addtogroup sensor Hub.
//! @ingroup sensorhub
//! @{
//
//*****************************************************************************

#ifndef APOLLO_DELAY2RUN_H
#define APOLLO_DELAY2RUN_H

// delay to run use the CTimer A0
#define APOLLO_DELAY2RUN_TIMERNUM 0
#define APOLLO_DELAY2RUN_TIMERSEG AM_HAL_CTIMER_TIMERA
#define APOLLO_DELAY2RUN_TIMERINT AM_HAL_CTIMER_INT_TIMERA0

// The frequency of 20 Hz, the period is 50ms
#define APOLLO_DELAY2RUN_FREQ    20

// Maximum number of delay to run functions
#define APOLLO_DELAY2RUN_FUNMAX  4


void delay2run_init(void);
void delay_to_run(uint16_t delayMs, void(*d2r_handle)(void*), void* inPara);

#endif // APOLLO_DELAY2RUN_H
