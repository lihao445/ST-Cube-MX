#include "tim_user.h"

float Vcx, Vcy, Vcr;

extern RC_ctrl_t rc_ctrl;

extern float Target_1, Target_2, Target_3, Target_4;
extern float Speed_Motor_Target_1, Speed_Motor_Target_2, Speed_Motor_Target_3, Speed_Motor_Target_4;
extern float Position_Motor_Target_1, Position_Motor_Target_2, Position_Motor_Target_3, Position_Motor_Target_4;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == htim1.Instance) // htim==(&htim1)
	{
		//			Speed_Motor_Target_1 = 300;
		//      Position_Motor_Target_1 = 8192 * 2 *19;

		RC_VC_DATA(); //将遥控器左边数据赋值

		if (rc_ctrl.rc.s[0] == 3) // S1中间
		{

			//				if(rc_ctrl.rc.s[1] == 1)    //S2上面    自动模式
			//				{
			//					if(rc_ctrl.rc.ch[2] == 0 | rc_ctrl.rc.ch[3] == 0 | rc_ctrl.rc.ch[4] ==0)
			//					{
			//						Chassis_Stop();
			//					}
			//					if(rc_ctrl.rc.ch[2] == 0 | rc_ctrl.rc.ch[3] > 0)
			//					{
			//						Chassis_Forward(rc_ctrl.rc.ch[3]);
			//					}
			//					if(rc_ctrl.rc.ch[2] == 0 | rc_ctrl.rc.ch[3] < 0)
			//					{
			//						Chassis_Reverse( - rc_ctrl.rc.ch[3]);
			//					}
			//					if(rc_ctrl.rc.ch[2] > 0 | rc_ctrl.rc.ch[3] == 0)
			//					{
			//						Chassis_Right(rc_ctrl.rc.ch[2]);
			//					}
			//					if(rc_ctrl.rc.ch[2] < 0 | rc_ctrl.rc.ch[3] == 0)
			//					{
			//						Chassis_Left( - rc_ctrl.rc.ch[2]);
			//					}
			//					if(rc_ctrl.rc.ch[4] > 0)
			//					{
			//						Chassis_Ring_Right(rc_ctrl.rc.ch[2]);
			//					}
			//					if(rc_ctrl.rc.ch[4] < 0)
			//					{
			//						Chassis_Ring_Left( - rc_ctrl.rc.ch[4]);
			//					}
			//				}

			if (rc_ctrl.rc.s[1] == 3) // S2中间    手动模式
			{
				Speed_Motor_Target_1 =   Vcx + Vcy - Vcr;
				Speed_Motor_Target_2 =   Vcx - Vcy - Vcr;
				Speed_Motor_Target_3 = - Vcx - Vcy - Vcr;
				Speed_Motor_Target_4 = - Vcx + Vcy - Vcr;
			}
		}

		Target_1 = PID_velocity_realize_1(Speed_Motor_Target_1, 1);
		Target_2 = PID_velocity_realize_1(Speed_Motor_Target_2, 2);
		Target_3 = PID_velocity_realize_1(Speed_Motor_Target_3, 3);
		Target_4 = PID_velocity_realize_1(Speed_Motor_Target_4, 4);
		//	 		Target_1 = pid_call_1(Position_Motor_Target_1,1);
		CAN_cmd_chassis(Target_1, Target_2, Target_3, Target_4);
	}
}

/*

V1 =   Vcx + Vcy   (c是Chassis)
V2 =   Vcx - Vcy
V3 = - Vcx - Vcy
V4 = - Vcx + Vcy

*/

void RC_VC_DATA(void)
{
	Vcx = 0.707106f * rc_ctrl.rc.ch[2];
	Vcy = 0.707106f * rc_ctrl.rc.ch[3];
	Vcr = 0.707106f * rc_ctrl.rc.ch[4];

	Vcx *= 7;
	Vcy *= 7;
	Vcr *= 7;
}

void Chassis_Stop(void)
{
	Vcx = 0;
	Vcy = 0;
	Speed_Motor_Target_1 = Vcx + Vcy;
	Speed_Motor_Target_2 = Vcx - Vcy;
	Speed_Motor_Target_3 = -Vcx - Vcy;
	Speed_Motor_Target_4 = -Vcx + Vcy;
}

void Chassis_Forward(float Chassis_Target)
{
	Vcx = 0;
	Vcy = Chassis_Target;
	Speed_Motor_Target_1 = Vcx + Vcy;
	Speed_Motor_Target_2 = Vcx - Vcy;
	Speed_Motor_Target_3 = -Vcx - Vcy;
	Speed_Motor_Target_4 = -Vcx + Vcy;
}

void Chassis_Reverse(float Chassis_Target)
{
	Vcx = 0;
	Vcy = -Chassis_Target;
	Speed_Motor_Target_1 = Vcx + Vcy;
	Speed_Motor_Target_2 = Vcx - Vcy;
	Speed_Motor_Target_3 = -Vcx - Vcy;
	Speed_Motor_Target_4 = -Vcx + Vcy;
}

void Chassis_Left(float Chassis_Target)
{
	Vcx = -Chassis_Target;
	Vcy = 0;
	Speed_Motor_Target_1 = Vcx + Vcy;
	Speed_Motor_Target_2 = Vcx - Vcy;
	Speed_Motor_Target_3 = -Vcx - Vcy;
	Speed_Motor_Target_4 = -Vcx + Vcy;
}

void Chassis_Right(float Chassis_Target)
{
	Vcx = Chassis_Target;
	Vcy = 0;
	Speed_Motor_Target_1 = Vcx + Vcy;
	Speed_Motor_Target_2 = Vcx - Vcy;
	Speed_Motor_Target_3 = -Vcx - Vcy;
	Speed_Motor_Target_4 = -Vcx + Vcy;
}

void Chassis_Ring_Left(float Chassis_Target)
{

	Speed_Motor_Target_1 = -Chassis_Target;
	Speed_Motor_Target_2 = -Chassis_Target;
	Speed_Motor_Target_3 = -Chassis_Target;
	Speed_Motor_Target_4 = -Chassis_Target;
}

void Chassis_Ring_Right(float Chassis_Target)
{

	Speed_Motor_Target_1 = Chassis_Target;
	Speed_Motor_Target_2 = Chassis_Target;
	Speed_Motor_Target_3 = Chassis_Target;
	Speed_Motor_Target_4 = Chassis_Target;
}
