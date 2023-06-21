/***      �� ���ֽǶȼ��� ��

 *      ������       ������ + +
 *   �������� �ة��������������� �ة�����++
 *   ��                 ��
 *   ��       ������       ��++ + + +
 *   ������������������������������������+
 *   ��                 ��+
 *   ��      ���ة�        ��
 *   ��                 ��
 *   ����������         ����������
 *       ��         ��
 *       ��         ��   + +
 *       ��         ��
 *       ��         ��������������������������������
 *       ��                        ��
 *       ��                        ������
 *       ��                        ������
 *       ��                        ��
 *       ������  ��  �����������������Щ�����  ��������  + + + +
 *         �� ���� ����       �� ���� ����
 *         �������ة�����       �������ة�����  + + + +
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






fp32 Radius = 0.707107f;    //��ת�ٶ�ϵ��
fp32 temp_x[4] = {0}, temp_y[4] = {0};    //ת����Ҫ��atan2����ķ�λ
fp32 helm_atan_angle[4];  					 //ͨ��atan2ת������ң�����Ƕ�


void helm_wheel_angle_calc(void)
{
	if (!(absolute_chassis_measure.Speed.vx == 0 && absolute_chassis_measure.Speed.vy == 0 && absolute_chassis_measure.Speed.vw == 0)) //��ֹ����Ϊ��
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
 * @brief  �Ƕ�ת��Ϊ����
 * @param  deg �Ƕ�ֵ
 * @retval rad ����ֵ
 * @attention
 */
fp32 deg2rad(fp32 deg)
{
    fp32 rad;
    rad = deg * (PI / 180.0f);
    return rad;
}

/**
 * @brief  ����ת��Ϊ�Ƕ�
 * @param  rad ����ֵ
 * @retval deg �Ƕ�ֵ
 * @attention
 */
fp32 rad2deg(fp32 rad)
{
    fp32 deg;
    deg = rad * (180.0f / PI);
    return deg;
}











////����ң����ң�˽Ƕ�
//float calc_angle_helm_wheel(float set_ch2, float set_ch3)
//{
//	if (ABS(set_ch2) > 120 || ABS(set_ch3) > 120)
//		return (atan2(-set_ch2, set_ch3)) * 180.00f / 3.14159f; // atan2����math.h
//	else
//		return 0.00f;
//}

////������Ӧ��ת����Ȧ������ֹ����
//float calc_motor_round_cnt(float angle, float last_angle)
//{
//	static float round_cnt = 0.00f;
//	if (angle - last_angle > 260) //��¼����ת��Ȧ��
//		round_cnt--;
//	else if (angle - last_angle < -260)
//		round_cnt++;
//	return round_cnt;
//}

////�Ż�����ת���Ƕȣ��ͽ�ԭ��
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
