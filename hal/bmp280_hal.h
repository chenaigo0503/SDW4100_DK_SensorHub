#ifndef _BMP280_HAL_H_
#define _BMP280_HAL_H_

#include "apollo_message.h"
#include "apollo_bmp280.h"
#include "apollo_tracelog.h"

extern uint8_t g_bmp280State;

void get_bmp280_send_msg(void);

#endif //  _BMP280_HAL_H_
