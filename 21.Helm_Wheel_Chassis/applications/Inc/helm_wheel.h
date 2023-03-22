#ifndef __HELM_WHEEL_H_
#define __HELM_WHEEL_H_
#include "include.h"

float calc_angle_helm(float set_ch2,float set_ch3);

float calc_motor_round_cnt(float angle,float last_angle);

float calc_min_angle(float set_angle,float last_set_angle);

#endif




