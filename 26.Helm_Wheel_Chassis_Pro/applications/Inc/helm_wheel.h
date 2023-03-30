#ifndef __HELM_WHEEL_H_
#define __HELM_WHEEL_H_
#include "include.h"

#define ABS(x)	((x>0) ? (x) : (-x))  //通用型绝对值函数(x是什么类型就是什么类型)

#define PI (3.141593)
#define Radian_to_Angle (180.0f / PI)
#define Angle_to_Radian (PI / 180.0f)

//底盘模式
typedef enum
{
    CHASSIS_GYROSCOPE = 0,	   //小陀螺模式
    CHASSIS_NORMAL   = 1,      //底盘遥控行走
} eChassisAction;

//底盘速度
typedef struct
{
    float vx;
    float vy;
    float vw;
} Chassis_Speed_t;

void Remote_Control_Chassis_Set_Mode(void);

void Remote_Control_Chassis_Mode(void);

void helm_wheel_angle_calc(Chassis_Speed_t *speed, fp32 *out_angle);

void helm_wheel_speed_calc(Chassis_Speed_t *speed, fp32 *out_speed);

void Chassis_Loop_Out(void);

float calc_min_angle(float set_angle, float last_set_angle);

//float calc_angle_helm_wheel(float set_ch2,float set_ch3);

//float calc_motor_round_cnt(float angle,float last_angle);

//float calc_min_angle(float set_angle,float last_set_angle);

#endif




