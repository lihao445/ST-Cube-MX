#ifndef __HELM_WHEEL_H_
#define __HELM_WHEEL_H_
#include "main.h"
#include "cmsis_os.h"
#include "can.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"
#include "can_receive.h"
#include "pid_user.h"
#include "remote_control.h"

#define PI (3.141592f)

//#define ABS(x)	((x>0) ? (x) : (-x))  //通用型绝对值函数(x是什么类型就是什么类型)

void helm_wheel_angle_calc(void);

void helm_wheel_speed_calc(void);

void get_helm_angle_pre(void);

void get_helm_total_angle(void);

void get_helm_proximity_angle(void);

void get_helm_reset_angle(void);

fp32 rad2deg(fp32 rad);

fp32 deg2rad(fp32 deg);

#endif




