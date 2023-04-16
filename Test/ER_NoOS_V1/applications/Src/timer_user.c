/**    ��ң�������ֿ��ơ�
 *           _____                    _____
 *          /\    \                  /\    \
 *         /::\    \                /::\    \
 *        /::::\    \              /::::\    \
 *       /::::::\    \            /::::::\    \
 *      /:::/\:::\    \          /:::/\:::\    \
 *     /:::/__\:::\    \        /:::/  \:::\    \
 *    /::::\   \:::\    \      /:::/    \:::\    \
 *   /::::::\   \:::\    \    /:::/    / \:::\    \
 *  /:::/\:::\   \:::\____\  /:::/    /   \:::\    \
 * /:::/  \:::\   \:::|    |/:::/____/     \:::\____\
 * \::/   |::::\  /:::|____|\:::\    \      \::/    /
 *  \/____|:::::\/:::/    /  \:::\    \      \/____/
 *        |:::::::::/    /    \:::\    \
 *        |::|\::::/    /      \:::\    \
 *        |::| \::/____/        \:::\    \
 *        |::|  ~|               \:::\    \
 *        |::|   |                \:::\    \
 *        \::|   |                 \:::\____\
 *         \:|   |                  \::/    /
 *          \|___|                   \/____/
 */
#include "timer_user.h"

extern RC_ctrl_t rc_ctrl;

fp32 res = 0.00f, prev_res = 0.00f, res1 = 0.00f;
fp32 round_cnt = 0.00f;
fp32 round1 = 0.00f, round2 = 0.00f;
fp32 direction_coefficient = 1.00f;

fp32 Angle_Helm_Target[4];
fp32 Speed_Motor_Target[4];
fp32 Shoot_Motor_Target[2];
fp32 Claw_Motor_Target[2];
fp32 Conveyer_Motor_Target[1];

fp32 M2006_Target[4];
fp32 M3508_Target[9];

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM1)
	{
		
		/***************************************�����˶�****************************************************************/
			res = calc_angle_helm_wheel(rc_ctrl.rc.ch[2], rc_ctrl.rc.ch[3]); //����ң����ң�˽Ƕ�
			res1 = calc_min_angle(res, prev_res);							 //�ͽ�תȦ��Ŀ��Ƕ�

			round_cnt = calc_motor_round_cnt((res + res1), prev_res); //����Ȧ��
			round1 = fabs(prev_res - res1);
			round2 = fabs(res - prev_res);
			if (round1 >= round2)
			{
				res1 = 0.00f;
				direction_coefficient = 1.00f;
			}
			if (round1 < round2)
			{
				res = 0.00f;
				direction_coefficient = -1.00f;
			}
			prev_res = res + res1;																				//��¼��һ�εĽǶ�����
			Angle_Helm_Target[0] = 67.00f / 20.00f * (36.00f * 8192.00f * (round_cnt + (res + res1) / 360.00f)); //������ת��������ת���ĽǶ�����
			Angle_Helm_Target[1] = 67.00f / 20.00f * (36.00f * 8192.00f * (round_cnt + (res + res1) / 360.00f));
			Angle_Helm_Target[2] = 67.00f / 20.00f * (36.00f * 8192.00f * (round_cnt + (res + res1) / 360.00f));
			Angle_Helm_Target[3] = 67.00f / 20.00f * (36.00f * 8192.00f * (round_cnt + (res + res1) / 360.00f));

			if (ABS(rc_ctrl.rc.ch[2]) > 250 || ABS(rc_ctrl.rc.ch[3]) > 250)
			{
				Speed_Motor_Target[0] = direction_coefficient * (-0.849106f * sqrt(rc_ctrl.rc.ch[2] * rc_ctrl.rc.ch[2] + rc_ctrl.rc.ch[3] * rc_ctrl.rc.ch[3]));
				Speed_Motor_Target[1] = direction_coefficient * (-0.849106f * sqrt(rc_ctrl.rc.ch[2] * rc_ctrl.rc.ch[2] + rc_ctrl.rc.ch[3] * rc_ctrl.rc.ch[3]) - rc_ctrl.rc.ch[3] / 98.636f);
				Speed_Motor_Target[2] = direction_coefficient * (-0.849106f * sqrt(rc_ctrl.rc.ch[2] * rc_ctrl.rc.ch[2] + rc_ctrl.rc.ch[3] * rc_ctrl.rc.ch[3]) - rc_ctrl.rc.ch[3] / 98.636f);
				Speed_Motor_Target[3] = direction_coefficient * (-0.849106f * sqrt(rc_ctrl.rc.ch[2] * rc_ctrl.rc.ch[2] + rc_ctrl.rc.ch[3] * rc_ctrl.rc.ch[3]));

				Speed_Motor_Target[0] *= 7;
				Speed_Motor_Target[1] *= 7;
				Speed_Motor_Target[2] *= 7;
				Speed_Motor_Target[3] *= 7;
			}
			if (ABS(rc_ctrl.rc.ch[2]) <= 250 && ABS(rc_ctrl.rc.ch[3]) <= 250)
			{
				Speed_Motor_Target[0] = 0;
				Speed_Motor_Target[1] = 0;
				Speed_Motor_Target[2] = 0;
				Speed_Motor_Target[3] = 0;
			}

			if (ABS(rc_ctrl.rc.ch[4]) >= 200)
			{
				Angle_Helm_Target[0] = -123494.400f + 67.00f / 20.00f * round_cnt * 8192.00f * 36.00f;
				Angle_Helm_Target[1] = 123494.400f + 67.00f / 20.00f * round_cnt * 8192.00f * 36.00f;
				Angle_Helm_Target[2] = -123494.400f + 67.00f / 20.00f * round_cnt * 8192.00f * 36.00f;
				Angle_Helm_Target[3] = 123494.400f + 67.00f / 20.00f * round_cnt * 8192.00f * 36.00f;
			}
			if (ABS(rc_ctrl.rc.ch[4]) >= 330)
			{
				Speed_Motor_Target[0] = +0.707106f * rc_ctrl.rc.ch[4]; // ԭ����ת
				Speed_Motor_Target[1] = -0.707106f * rc_ctrl.rc.ch[4];
				Speed_Motor_Target[2] = -0.707106f * rc_ctrl.rc.ch[4];
				Speed_Motor_Target[3] = +0.707106f * rc_ctrl.rc.ch[4];

				Speed_Motor_Target[0] *= 5.2f;
				Speed_Motor_Target[1] *= 5.2f;
				Speed_Motor_Target[2] *= 5.2f;
				Speed_Motor_Target[3] *= 5.2f;
			}
			for(int i = 0;i < 4;i++)
			{
				M3508_Target[i] = PID_velocity_realize_1(Speed_Motor_Target[i],i+1);
				M2006_Target[i] = pid_call_2(Angle_Helm_Target[i],i+1);
			}
			CAN1_CMD_1(M3508_Target[0], M3508_Target[1], M3508_Target[2], M3508_Target[3]);
			CAN2_CMD_1(M2006_Target[0], M2006_Target[1], M2006_Target[2], M2006_Target[3]);
		
		

		if (rc_ctrl.rc.s[0] == 1 && rc_ctrl.rc.s[1] == 3) //�����ջ�ģʽ
		{
		/**********************************צ��****************************************************************/
			if(ABS(rc_ctrl.rc.ch[1]) <= 180)
			{
				Claw_Motor_Target[0] = -(fp32)rc_ctrl.rc.ch[0] * 1.0f;
				M3508_Target[4] = PID_velocity_realize_1(Claw_Motor_Target[0],5);
			}
			if(ABS(rc_ctrl.rc.ch[0]) <= 180)
			{
				if(ABS(rc_ctrl.rc.ch[1]) >= 300 )
				{
				Claw_Motor_Target[1] = (fp32)rc_ctrl.rc.ch[1] * 1.0f;
				}
				else if(ABS(rc_ctrl.rc.ch[1]) <= -300 )
				{
				Claw_Motor_Target[1] = (fp32)rc_ctrl.rc.ch[1] * 1.0f;
				}
				else if(ABS(rc_ctrl.rc.ch[1]) == 0 )
				{
					Claw_Motor_Target[1] = (fp32)rc_ctrl.rc.ch[1] * 0.0f;
				}
				M3508_Target[5] = PID_velocity_realize_1(Claw_Motor_Target[1],6);
			}
			
//			if(rc_ctrl.rc.ch[1] >= 580)
//			{
//					HAL_GPIO_WritePin(Claw_Catch_GPIO_Port, Claw_Catch_Pin, GPIO_PIN_SET);
//			}			
//			else if(rc_ctrl.rc.ch[1] <= -580)
//			{
//				  HAL_GPIO_WritePin(Claw_Catch_GPIO_Port, Claw_Catch_Pin, GPIO_PIN_RESET);
//			}
		}
		
		if (rc_ctrl.rc.s[0] == 2 && rc_ctrl.rc.s[1] == 3) //���̷���ģʽ
		{
				/****************************************����װ��****************************************************************/
			Shoot_Motor_Target[0] =  rc_ctrl.rc.ch[1] * 13.67;
			Shoot_Motor_Target[1] = -rc_ctrl.rc.ch[1] * 13.67;
			if(rc_ctrl.rc.ch[0] >= 580)
			{
					Conveyer_Motor_Target[0] = 5000;
			}
			else if(rc_ctrl.rc.ch[0] <= -580)
			{
					Conveyer_Motor_Target[0] = 0;
			}

			for(int i = 6;i < 8;i++)
			{
				M3508_Target[i] = PID_velocity_realize_2(Shoot_Motor_Target[i-6],i-5);
			}
			M3508_Target[8] = PID_velocity_realize_2(Conveyer_Motor_Target[0],3);
		}
			CAN1_CMD_2(M3508_Target[4],M3508_Target[5],0,0);   //5,6�ŵ��
			CAN2_CMD_2(M3508_Target[6],M3508_Target[7],M3508_Target[8],0);   //CAN2��1,2,3�ŵ��
	}
}
