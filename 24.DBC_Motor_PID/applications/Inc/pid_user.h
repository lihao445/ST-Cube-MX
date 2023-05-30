#ifndef __PID_USER_H
#define __PID_USER_H
#include "pid.h"
#include "include.h"

void PID_devices_Init(void);

fp32 PID_velocity_realize(fp32 set_speed);
fp32 PID_position_realize(fp32 set_pos);
fp32 PID_call_realize(fp32 position);


#endif























