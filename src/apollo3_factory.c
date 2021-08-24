
#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"
#include "apollo_tracelog.h"
#include "apollo3_init.h"
#include "apollo3_uart.h"
#include "apollo3_amotas.h"
#include "apollo3_ios.h"
#include "apollo3_rtc.h"
#include "apollo_delay2run.h"
#include "apollo_message.h"
#include "lsm6dso_hal.h"
#include "bmp280_hal.h"
#include "ak09918_hal.h"
#include "pah8011_hal.h"

extern uint8_t g_UARTRxBuf[128];
extern uint8_t g_UARTRxBufLen;

static uint8_t strtou8(char* nptr)
{
    uint8_t ret;

    if (*nptr < '0' || *nptr > '9')
        return 0;

    ret = (*nptr - '0') * 10;
    nptr++;

    if (*nptr < '0' || *nptr > '9')
        return 0;

    ret += *nptr - '0';
    return ret;
}

static void send_test_msg(char* nptr)
{
    uart_print("###");
    uart_print(nptr);
    uart_print("***\r\n");
}

// For Factory Test
void factory_test(void)
{
    if (g_UARTRxBufLen > 0)
    {
        uint32_t i = 0;
        char* reciveCmd = NULL;
        char* reciveCmdTail = NULL;
        uint8_t testCmd;
        char msgBuf[128];
        float lsm6dData[3] = {0};
        int16_t compass_data[3];
        int16_t compass_st[2];

        // Wait for a frame to complete.
        am_util_delay_ms(3);
        reciveCmd = strstr((const char*)g_UARTRxBuf, "###");
        if (reciveCmd != NULL)
        {
            reciveCmdTail = strstr((const char*)g_UARTRxBuf, "***");
            if (reciveCmdTail != NULL)
            {
                if(reciveCmdTail - reciveCmd == 5)
                {
                    reciveCmd += 3;
                    *reciveCmdTail = '\0';
                    testCmd = strtou8(reciveCmd);
                    switch (testCmd)
                    {
                        case 1:
                            PR_INFO("MCU status test.");
                            send_test_msg("");
                            break;
                        case 2:
                            PR_INFO("MCU version check.");
                            am_util_stdio_sprintf(msgBuf,"%02X.%02X.%02X", sw_version[0], APOLLO3_HUB_VER1, APOLLO3_HUB_VER2);
                            send_test_msg(msgBuf);
                        case 3:
                            PR_INFO("Gravity sensor test.");
                            for (i = 0; i < 800; i++)
                            {
                                if (!lsm6dso_acceleration_get(lsm6dData))
                                    break;
                                am_util_delay_ms(10);
                            }
                            if (i == 800)
                            {
                                PR_ERR("Gravity sensor test ERROR.");
                                send_test_msg("Gravity sensor failed to pick up data.");
                            }
                            else
                            {
                                PR_INFO("Gravity sensor test successed.");
                                am_util_stdio_sprintf(msgBuf, "Gravity-value: x = %4.2f, y = %4.2f, z = %4.2f",
                                        lsm6dData[0], lsm6dData[1], lsm6dData[2]);
                                send_test_msg(msgBuf);
                            }
                            break;
                        case 4:
                            PR_INFO("Gyroscope sensor test.");
                            for (i = 0; i < 800; i++)
                            {
                                if (!lsm6dso_angular_get(lsm6dData))
                                    break;
                                am_util_delay_ms(10);
                            }
                            if (i == 800)
                            {
                                PR_ERR("Gyroscope sensor test ERROR.");
                                send_test_msg("Gyroscope sensor failed to pick up data.");
                            }
                            else
                            {
                                PR_INFO("Gyroscope sensor test successed.");
                                am_util_stdio_sprintf(msgBuf, "Gyroscope-value: x = %4.2f, y = %4.2f, z = %4.2f",
                                        lsm6dData[0], lsm6dData[1], lsm6dData[2]);
                                send_test_msg(msgBuf);
                            }
                            break;
                        case 5:
                            PR_INFO("Compass sensor test.");
                            ak099xx_start(10);
                            for (i = 0; i < 800; i++)
                            {
                                am_util_delay_ms(10);
                                ak099xx_get_data(compass_data, compass_st);
                                if (compass_st[1] &= 0x0004)
                                    break;
                            }
                            if (i == 800)
                            {
                                PR_ERR("Compass sensor test ERROR.");
                                send_test_msg("Compass sensor failed to pick up data.");
                            }
                            else
                            {
                                PR_INFO("Compass sensor test successed.");
                                am_util_stdio_sprintf(msgBuf, "Compass-value: x = %d, y = %d, z = %d",
                                        compass_data[0], compass_data[1], compass_data[2]);
                                send_test_msg(msgBuf);
                            }
                            ak099xx_stop();
                            break;
                        case 6:
                            PR_INFO("Temperature sensor test.");
                            bmp280_set_power_mode(BMP280_NORMAL_MODE);
                            for (i = 0; i < 800; i++)
                            {
                                struct bmp280_status m_statue;
                                am_util_delay_ms(10);
                                bmp280_get_status(&m_statue);
                                if (m_statue.im_update && m_statue.measuring)
                                {
                                    struct bmp280_uncomp_data m_uncomp_data;
                                    int32_t temp = 0;

                                    bmp280_get_uncomp_data(&m_uncomp_data);
                                    bmp280_get_comp_temp_32bit(&temp, m_uncomp_data.uncomp_temp);
                                    am_util_stdio_sprintf(msgBuf, "Temperature-value: %d", temp);
                                    send_test_msg(msgBuf);
                                    break;
                                }
                            }
                            if (i == 800)
                            {
                                PR_ERR("Temperature sensor test ERROR.");
                                send_test_msg("Temperature sensor failed to pick up data.");
                            }
                            bmp280_set_config();
                            break;
                        case 7:
                            PR_INFO("Pressure sensor test.");
                            bmp280_set_power_mode(BMP280_NORMAL_MODE);
                            for (i = 0; i < 800; i++)
                            {
                                struct bmp280_status m_statue;
                                am_util_delay_ms(10);
                                bmp280_get_status(&m_statue);
                                if (m_statue.im_update && m_statue.measuring)
                                {
                                    struct bmp280_uncomp_data m_uncomp_data;
                                    uint32_t press;

                                    bmp280_get_uncomp_data(&m_uncomp_data);
                                    bmp280_get_comp_pres_32bit(&press, m_uncomp_data.uncomp_press);
                                    am_util_stdio_sprintf(msgBuf, "Pressure-value: %d", press);
                                    send_test_msg(msgBuf);
                                    break;
                                }
                            }
                            if (i == 800)
                            {
                                PR_ERR("Pressure sensor test ERROR.");
                                send_test_msg("Pressure sensor failed to pick up data.");
                            }
                            bmp280_set_config();
                            break;
                        case 8:
                            PR_INFO("Heart-Rate sensor test.");
                            pah8series_ppg_dri_HRD_init();
                            break;

                        default:
                            PR_ERR("The test command: %d, does not exist.", testCmd);
                            send_test_msg("The test command does not exist.");
                            break;
                    }
                }
                else
                {
                    PR_ERR("Command length incorrect.");
                }
            }
            else
            {
                PR_ERR("Did not receive the correct command tail.");
            }
        }
        else
        {
            PR_ERR("Did not receive the correct command header.");
        }
        g_UARTRxBufLen = 0;
    }
}
