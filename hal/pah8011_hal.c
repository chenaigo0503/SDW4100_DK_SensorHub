#include "pah8011_hal.h"

extern volatile bool has_interrupt_button;
extern volatile bool has_interrupt_pah;
extern uint8_t touch_stat;
extern volatile uint64_t interrupt_pah_timestamp;
extern float hr;
extern int32_t hr_trust_level;
extern int16_t grade;
extern uint8_t hr_stat;

void get_hr_touch_send_msg(void)
{
    pah8series_touch_mode_dri_task(&has_interrupt_pah ,&has_interrupt_button ,&interrupt_pah_timestamp);

    if (touch_stat & 0x10)
    {
        touch_stat &= 0x0F;
        PR_INFO("%s, get touch: %d", __func__, touch_stat);
        send_event_msg_to_host(APOLLO_SENSOR_8_EVNT, 1, (uint8_t*)&touch_stat);
        inform_host();
    }
}

void get_irg_hr_send_msg(void)
{
    uint8_t hr_data[10] = {0};
    /**pah8series_ppg_dri_HRD_task, this function only read ppg data & set raw_data format,
     * Customer can run this function on interrupt_handler or while loop. */
    pah8series_ppg_dri_HRD_task(&has_interrupt_pah ,&has_interrupt_button ,&interrupt_pah_timestamp);

    /**hrd_alg_task, this function will run pixart alg, Customer need run this function on
     * low priority task before next pah8series_ppg_dri_HRD_task() */
    hrd_alg_task();

    if (hr_stat == MSG_HR_READY)
    {
        hr_stat = 0;
        *(int32_t*)(&hr_data[0]) = hr;
        *(int32_t*)(&hr_data[4]) = hr_trust_level;
        *(int16_t*)(&hr_data[8]) = grade;

        send_event_msg_to_host(APOLLO_SENSOR_5_EVNT, 10, (uint8_t*)&hr_data);
        inform_host();
    }
}
