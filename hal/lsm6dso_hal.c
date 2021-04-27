#include "lsm6dso_hal.h"

extern stmdev_ctx_t g_Lsm6dsoCtx;

void stepcounter_reset(void)
{
    lsm6dso_steps_reset(&g_Lsm6dsoCtx);
}

void get_acc_send_msg(void)
{
    float accData[3] = {0};

    if (!lsm6dso_acceleration_get(accData))
    {
        send_event_msg_to_host(APOLLO_SENSOR_0_EVNT, 12, (uint8_t*)accData);
        PR_INFO("x1=%f,x2=%f,x3=%f\r\n", accData[0], accData[1], accData[2]);

        inform_host();
        wait_fifo_empty();
    }
}

void get_gyro_send_msg(void)
{
    float gyroData[3] = {0};

    if (!lsm6dso_angular_get(gyroData))
    {
        PR_INFO("x1=%4.02f,x2=%4.02f,x3=%4.02f\r\n", gyroData[0], gyroData[1], gyroData[2]);
        send_event_msg_to_host(APOLLO_SENSOR_1_EVNT, 12, (uint8_t*)gyroData);
        inform_host();
        wait_fifo_empty();
    }
}

void get_step_send_msg(void)
{
    uint16_t step;
    static uint16_t last_steps;

    step = lsm6dso_step_get();

    if (last_steps != step)
    {
        PR_INFO("%s, get stepcounter: %d", __func__, step);
        send_event_msg_to_host(APOLLO_SENSOR_6_EVNT, 2, (uint8_t*)&step);
        inform_host();
        last_steps = step;
    }
}

void get_tilt_status_send_msg(void)
{
    float accData[3] = {0};
    static uint8_t lift_wrist_state = 0;

    if (lsm6dso_tilt_status())
    {
        if (!lsm6dso_acceleration_get(accData))
        {
            if (accData[2] < -860)
            {
                if (lift_wrist_state == 0)
                {
                    lift_wrist_state = 1;
                    PR_INFO("%s, Lift the wrist.", __func__);
                    send_event_msg_to_host(APOLLO_SENSOR_7_EVNT, 1, (uint8_t*)&lift_wrist_state);
                    inform_host();
                }
            }
            else if (accData[0] < -920 || accData[0] > 920)
            {
                if (lift_wrist_state == 1)
                {
                    lift_wrist_state = 0;
                    PR_INFO("%s, put down the wrist.", __func__);
                    send_event_msg_to_host(APOLLO_SENSOR_7_EVNT, 1, (uint8_t*)&lift_wrist_state);
                    inform_host();
                }
            }
        }
    }
}
