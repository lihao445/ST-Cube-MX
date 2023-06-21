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




void helm_wheel_speed_calc(void)
{
    fp32 wheel_rpm_ratio; //�����ٶȱ���

    //    wheel_rpm_ratio = 60.0f / (WHEEL_PERIMETER * 3.14159f) * CHASSIS_DECELE_RATIO * 1000;
    wheel_rpm_ratio = 1;

    Speed_Motor_Target[0] = sqrt(pow(absolute_chassis_measure.Speed.vy - absolute_chassis_measure.Speed.vw * Radius * 0.707107f, 2) + pow(absolute_chassis_measure.Speed.vx + absolute_chassis_measure.Speed.vw * Radius * 0.707107f, 2)) * wheel_rpm_ratio; //��η���ƽ����
    Speed_Motor_Target[1] = sqrt(pow(absolute_chassis_measure.Speed.vy - absolute_chassis_measure.Speed.vw * Radius * 0.707107f, 2) + pow(absolute_chassis_measure.Speed.vx - absolute_chassis_measure.Speed.vw * Radius * 0.707107f, 2)) * wheel_rpm_ratio;
    Speed_Motor_Target[2] = sqrt(pow(absolute_chassis_measure.Speed.vy - absolute_chassis_measure.Speed.vw * Radius * 0.707107f, 2) + pow(absolute_chassis_measure.Speed.vx + absolute_chassis_measure.Speed.vw * Radius * 0.707107f, 2)) * wheel_rpm_ratio;
    Speed_Motor_Target[3] = sqrt(pow(absolute_chassis_measure.Speed.vy + absolute_chassis_measure.Speed.vw * Radius * 0.707107f, 2) + pow(absolute_chassis_measure.Speed.vx + absolute_chassis_measure.Speed.vw * Radius * 0.707107f, 2)) * wheel_rpm_ratio;
}


fp32 helm_atan_angle[4];
fp32 helm_atan_angle_pre[4];

fp32 helm_angle_total[4];
fp32 helm_angle_total_pre[4];

void get_helm_angle_pre(void)
{
	for(int i = 0;i < 4;i++)
	{
		helm_atan_angle_pre[i] = helm_atan_angle[i];
		helm_angle_total_pre[i] = helm_angle_total[i];
	}
}


fp32 round_cnt[4] = {0};
//fp32 helm_angle_total[4];

void get_helm_total_angle(void)
{
	for(int i = 0;i < 4;i++)
	{
		if((helm_atan_angle[i] - helm_atan_angle_pre[i]) >= 300.0f)
		{
			round_cnt[i]--;
		}
		else if((helm_atan_angle[i] - helm_atan_angle_pre[i]) <= -300.0f)
		{
			round_cnt[i]++;
		}
		helm_angle_total[i] = helm_atan_angle[i] + round_cnt[i] * 360.0f;
	}
}


void get_helm_proximity_angle(void)
{
	for(int i = 0;i < 4;i++)
	{
		if((helm_angle_total[i] - helm_angle_total_pre[i]) >= 90.0f)
		{
			helm_angle_total[i] -= 180.0f;
		}
		else if((helm_angle_total[i] - helm_angle_total_pre[i]) <= -90.0f)
		{
			helm_angle_total[i] += 180.0f;
		}
		else
		{
//			helm_angle_total[i] = helm_angle_total[i];
		}
	}
}



void get_helm_reset_angle(void)
{
	if ((absolute_chassis_measure.Speed.vx == 0 && absolute_chassis_measure.Speed.vy == 0 && absolute_chassis_measure.Speed.vw == 0))
	{
		for(int i = 0;i < 4;i++)
		{
			if(((round_cnt[i] * 360.0f - 90.0f) <= helm_angle_total_pre[i] ) && (helm_angle_total_pre[i] <= (round_cnt[i] * 360.0f + 90.0f)))
			{
				helm_angle_total[i] = round_cnt[i] * 360.0f;
			}
			else
			{
				helm_angle_total[i] = round_cnt[i] * 180.0f;
			}
		}
	}
}


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
		
		get_helm_angle_pre();
		
		for (int i = 0; i < 4; i++)
    {
      helm_atan_angle[i] = atan2(temp_x[i], temp_y[i]);
      helm_atan_angle[i] = rad2deg(helm_atan_angle[i]);
    }
		
		get_helm_total_angle();
		
		get_helm_proximity_angle();
	}
	get_helm_reset_angle();
	
	for(int i = 0;i < 4;i++)
	{
		Angle_Helm_Target[i] = (helm_angle_total[i] / 360.0f * 8191.0f) * 36.0f * (67.0f / 20.0f);
	}
	
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
