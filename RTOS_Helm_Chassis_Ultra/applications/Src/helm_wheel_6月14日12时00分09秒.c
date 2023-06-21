/***      《 舵轮角度计算 》

 *      ┌─┐       ┌─┐ + +
 *   ┌──┘ ┴───────┘ ┴──┐++
 *   │                 │
 *   │       ───       │++ + + +
 *   ███████───███████│+
 *   │                 │+
 *   │      ─┴─        │
 *   │                 │
 *   └───┐         ┌───┘
 *       │         │
 *       │         │   + +
 *       │         │
 *       │         └──────────────┐
 *       │                        │
 *       │                        ├─┐
 *       │                        ┌─┘
 *       │                        │
 *       └─┐  ┐  ┌───────┬──┐  ┌──┘  + + + +
 *         │ ─┤ ─┤       │ ─┤ ─┤
 *         └──┴──┘       └──┴──┘  + + + +
 *
 *
 */
#include "helm_wheel.h"
#include "chassis_api.h"
#include <math.h>

extern RC_ctrl_t rc_ctrl;

extern Chassis_measure_t absolute_chassis_measure;

extern fp32 Angle_Helm_Target[4];
extern fp32 Speed_Motor_Target[4];






fp32 Radius = 0.707107f;    //旋转速度系数
fp32 temp_x[4] = {0}, temp_y[4] = {0};    //转化成要被atan2计算的方位
fp32 helm_atan_angle[4];  					 //通过atan2转换出的遥控器角度


void helm_wheel_angle_calc(void)
{
	if (!(absolute_chassis_measure.Speed.vx == 0 && absolute_chassis_measure.Speed.vy == 0 && absolute_chassis_measure.Speed.vw == 0)) //防止除数为零
  {
		temp_x[0] = absolute_chassis_measure.Speed.vx + absolute_chassis_measure.Speed.vw * Radius;
		temp_y[0] = absolute_chassis_measure.Speed.vy + absolute_chassis_measure.Speed.vw * Radius;
		temp_x[1] = absolute_chassis_measure.Speed.vx - absolute_chassis_measure.Speed.vw * Radius;
		temp_y[1] = absolute_chassis_measure.Speed.vy + absolute_chassis_measure.Speed.vw * Radius;
		temp_x[2] = absolute_chassis_measure.Speed.vx + absolute_chassis_measure.Speed.vw * Radius;
		temp_y[2] = absolute_chassis_measure.Speed.vy + absolute_chassis_measure.Speed.vw * Radius;
		temp_x[3] = absolute_chassis_measure.Speed.vx - absolute_chassis_measure.Speed.vw * Radius;
		temp_y[3] = absolute_chassis_measure.Speed.vy + absolute_chassis_measure.Speed.vw * Radius;
		
		for (int i = 0; i < 4; i++)
    {
      helm_atan_angle[i] = atan2(temp_x[i], temp_y[i]);
      helm_atan_angle[i] = rad2deg(helm_atan_angle[i]);
    }
	}
	
	
	
	
	
	
}


void helm_wheel_speed_calc(void)
{
	
}

void get_helm_angle_offset(void)



void get_helm_round_cnt(void)
{
	
}




/**
 * @brief  角度转换为弧度
 * @param  deg 角度值
 * @retval rad 弧度值
 * @attention
 */
fp32 deg2rad(fp32 deg)
{
    fp32 rad;
    rad = deg * (PI / 180.0f);
    return rad;
}

/**
 * @brief  弧度转换为角度
 * @param  rad 弧度值
 * @retval deg 角度值
 * @attention
 */
fp32 rad2deg(fp32 rad)
{
    fp32 deg;
    deg = rad * (180.0f / PI);
    return deg;
}











////计算遥控器遥杆角度
//float calc_angle_helm_wheel(float set_ch2, float set_ch3)
//{
//	if (ABS(set_ch2) > 120 || ABS(set_ch3) > 120)
//		return (atan2(-set_ch2, set_ch3)) * 180.00f / 3.14159f; // atan2函数math.h
//	else
//		return 0.00f;
//}

////计算电机应该转动的圈数，防止跳变
//float calc_motor_round_cnt(float angle, float last_angle)
//{
//	static float round_cnt = 0.00f;
//	if (angle - last_angle > 260) //记录轮子转动圈数
//		round_cnt--;
//	else if (angle - last_angle < -260)
//		round_cnt++;
//	return round_cnt;
//}

////优化舵轮转动角度，就近原则
//float calc_min_angle(float set_angle, float last_set_angle)
//{
//	float target_angle;
//	if (fabs(set_angle - last_set_angle) > 90)
//	{
//		if (set_angle >= 90 && set_angle <= 181)
//		{
//			target_angle = set_angle - 180.00f;
//		}
//		if (set_angle <= -90 && set_angle >= -181)
//		{
//			target_angle = set_angle + 180.00f;
//		}
//	}
//	else
//	{
//		target_angle = set_angle;
//	}
//	return target_angle;
//}
