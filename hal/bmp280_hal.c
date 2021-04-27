#include "bmp280_hal.h"

uint8_t g_bmp280State = 0; // 1: temp, 2: pres, 3: all

void get_bmp280_send_msg(void)
{
    struct bmp280_status m_statue;

    bmp280_get_status(&m_statue);

    if (m_statue.im_update && m_statue.measuring)
    {
        struct bmp280_uncomp_data m_uncomp_data;
        int32_t temp = 0;
        uint32_t press;
        uint8_t ret;

        bmp280_get_uncomp_data(&m_uncomp_data);

        if (g_bmp280State & 0x01) // temperature
        {
            bmp280_get_comp_temp_32bit(&temp, m_uncomp_data.uncomp_temp);

            PR_INFO("%s, temp = %d\r\n", __func__, temp);

            ret = send_event_msg_to_host(APOLLO_SENSOR_2_EVNT, 4, (uint8_t*)&temp);

            if (ret)
            {
                PR_ERR("Temperature error");
            }
            else
            {
                inform_host();
                wait_fifo_empty();
            }
        }

        if (g_bmp280State & 0x02) // pressure
        {
            bmp280_get_comp_pres_32bit(&press, m_uncomp_data.uncomp_press);

            ret = send_event_msg_to_host(APOLLO_SENSOR_3_EVNT, 4, (uint8_t*)&press);

            if (ret)
            {
                PR_ERR("Pressure error");
            }
            else
            {
                inform_host();
                wait_fifo_empty();
            }
        }
    }
}

