#include "ak09918_hal.h"

void get_magnet_send_msg(void)
{
    if (ak099xx_check_rdy())
    {
        int16_t ak_data[3];
        int16_t ak_st[2];
        ak099xx_get_data(ak_data, ak_st);

        if (ak_st[1] &= 0x0004)
        {
            send_event_msg_to_host(APOLLO_SENSOR_4_EVNT, 12, (uint8_t*)&ak_data);

            PR_INFO("x1=%d,x2=%d,x3=%d\r\n", ak_data[0], ak_data[1], ak_data[2]);
            inform_host();
            wait_fifo_empty();
        }
    }
}
