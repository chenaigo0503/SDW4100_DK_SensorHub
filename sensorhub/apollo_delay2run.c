//*****************************************************************************
//
//! @file apollo_delay2run.h
//!
//! @brief Use a timer to suspend a task before it executes.
//!
//! @addtogroup sensor Hub.
//! @ingroup sensorhub
//
//*****************************************************************************

#include <string.h>
#include "am_mcu_apollo.h"
#include "apollo_tracelog.h"
#include "apollo_delay2run.h"

static bool d2rTimerRun = false;
static struct
{
    uint16_t waitNum;
    void(*d2r_fun)(void*);
    void* inPara;
}d2r_array[APOLLO_DELAY2RUN_FUNMAX];

//*****************************************************************************
//
// Timer handling to delay2run
//
//*****************************************************************************
static am_hal_ctimer_config_t g_delay2runTimer =
{
    // Don't link timers.
    0,

    // Set up TimerA.
    (AM_HAL_CTIMER_FN_REPEAT |
     AM_HAL_CTIMER_INT_ENABLE |
     AM_HAL_CTIMER_HFRC_12KHZ),

    // No configuration for TimerB.
    0,
};

// delay to run handle
static void delay2run_handle(void)
{
    uint8_t i;
    bool d2rIs = false;
    for(i = 0; i < APOLLO_DELAY2RUN_FUNMAX; i++)
    {
        if (d2r_array[i].waitNum)
            d2r_array[i].waitNum--;

        if (d2r_array[i].waitNum == 0 && d2r_array[i].d2r_fun != NULL)
        {
            d2r_array[i].d2r_fun(d2r_array[i].inPara);
            d2r_array[i].d2r_fun = NULL;
            d2r_array[i].inPara = NULL;
        }
        else if (d2r_array[i].waitNum)
        {
            d2rIs = true;
        }
    }

    // Function processing completed, stop the timer
    if (!d2rIs)
    {
        // PR_INFO("Completed, stop the timer.");
        am_hal_ctimer_stop(APOLLO_DELAY2RUN_TIMERNUM, APOLLO_DELAY2RUN_TIMERSEG);
        d2rTimerRun = false;
    }
}

void delay2run_init(void)
{
    uint32_t timerPeriod;

    memset(d2r_array, 0, sizeof(d2r_array));

    am_hal_ctimer_clear(APOLLO_DELAY2RUN_TIMERNUM, APOLLO_DELAY2RUN_TIMERSEG);
    am_hal_ctimer_config(APOLLO_DELAY2RUN_TIMERNUM, &g_delay2runTimer);

    timerPeriod = 12000 / APOLLO_DELAY2RUN_FREQ;
    am_hal_ctimer_period_set(APOLLO_DELAY2RUN_TIMERNUM, APOLLO_DELAY2RUN_TIMERSEG,
                                timerPeriod, (timerPeriod >> 1));

    am_hal_ctimer_int_clear(APOLLO_DELAY2RUN_TIMERINT);
    am_hal_ctimer_int_register(AM_HAL_CTIMER_INT_TIMERA0,
                                delay2run_handle);
    am_hal_ctimer_int_enable(APOLLO_DELAY2RUN_TIMERINT);
    NVIC_EnableIRQ(CTIMER_IRQn);
}

void delay_to_run(uint16_t delayMs, void(*d2r_handle)(void*), void* inPara)
{
    uint8_t i;

    if (d2r_handle == NULL)
    {
        PR_ERR("ERROR The function is NULL.");
        return;
    }

    // Execute the function without delay.
    if (delayMs == 0)
    {
        d2r_handle(inPara);
        return;
    }

    for(i = 0; i < APOLLO_DELAY2RUN_FUNMAX; i++)
    {
        // add this can increase the execution time
        if (d2r_array[i].d2r_fun == d2r_handle)
        {
            d2r_array[i].waitNum = (delayMs / 50) + 1;
            if(delayMs % 50)
                d2r_array[i].waitNum++;
            // Parameters will be updated
            d2r_array[i].inPara = inPara;

            break;
        }
        else if (d2r_array[i].waitNum == 0 && d2r_array[i].d2r_fun == NULL)
        {
            d2r_array[i].waitNum = (delayMs / 50);
            if(delayMs % 50)
                d2r_array[i].waitNum++;
            d2r_array[i].d2r_fun = d2r_handle;
            d2r_array[i].inPara = inPara;

            break;
        }
    }

    // The function that needs to be executed is full and cannot be added
    if (i == APOLLO_DELAY2RUN_FUNMAX)
    {
        PR_ERR("ERROR The function that needs to be executed is full.");
        return;
    }

    // Start the timer
    // Just in case host died without sending STOP last time
    if(!d2rTimerRun)
    {
        am_hal_ctimer_stop(APOLLO_DELAY2RUN_TIMERNUM, APOLLO_DELAY2RUN_TIMERSEG);
        am_hal_ctimer_start(APOLLO_DELAY2RUN_TIMERNUM, APOLLO_DELAY2RUN_TIMERSEG);
        d2rTimerRun = true;
    }
}

// Timer Interrupt Service Routine (ISR)
void am_ctimer_isr(void)
{
    uint32_t ui32Status;

    ui32Status = am_hal_ctimer_int_status_get(false);
    am_hal_ctimer_int_clear(ui32Status);

    am_hal_ctimer_int_service(ui32Status);
}
