#ifndef _LSM6DSO_HAL_H_
#define _LSM6DSO_HAL_H_

#include "apollo_lsm6dso.h"
#include "apollo_message.h"
#include "apollo_tracelog.h"

void stepcounter_reset(void);
void get_acc_send_msg(void);
void get_gyro_send_msg(void);
void get_step_send_msg(void);
void get_tilt_status_send_msg(void);

#endif  // _LSM6DSO_HAL_H_
