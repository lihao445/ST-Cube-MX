#include "tim_user.h"

float Vcx,Vcy,Wcr;
float a,b;

extern RC_ctrl_t rc_ctrl;

extern float Target_1,Target_2,Target_3,Target_4;
extern float Speed_Motor_Target_1,Speed_Motor_Target_2,Speed_Motor_Target_3,Speed_Motor_Target_4;
extern float Position_Motor_Target_1,Position_Motor_Target_2,Position_Motor_Target_3,Position_Motor_Target_4;


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{		
		if(htim->Instance == htim1.Instance)               //htim==(&htim1)
		{
//			Speed_Motor_Target_1 = 300;
//      Position_Motor_Target_1 = 8192 * 2 *19;
			
			RC_VC_DATA();         //将遥控器左边数据赋值
			
			if(rc_ctrl.rc.s[0] == 3)       //S1中间
			{
					
				if(rc_ctrl.rc.s[1] == 3)    //S2中间    手动模式
				{
					Mecanum_Wheel_Speed_Calculation();
				}
			}
			Transmit_Speed();
		}
}












/*

V1 = - Vcx + Vcy + Wcr * ( a + b )        (c是Chassis)
V2 =   Vcx + Vcy - Wcr * ( a + b )
V3 = - Vcx + Vcy - Wcr * ( a + b )
V4 =   Vcx + Vcy + Wcr * ( a + b )

*/

void RC_VC_DATA(void)
{
	Vcx = 0.707106f * rc_ctrl.rc.ch[2];
	Vcy = 0.707106f * rc_ctrl.rc.ch[3];
	Wcr = 0.707106f * rc_ctrl.rc.ch[4];
	
	Vcx *= 7;
	Vcy *= 7;
	Wcr *= -7;
	
	a =  0.085;           //轮子到底盘中心的 X轴距离
	b =  0.065;					  //轮子到底盘中心的 Y轴距离
}


void Mecanum_Wheel_Speed_Calculation(void)
{
	Speed_Motor_Target_1 = - Vcx + Vcy + Wcr * ( a + b ) ;
	Speed_Motor_Target_2 =   Vcx + Vcy - Wcr * ( a + b ) ;
	Speed_Motor_Target_3 = - Vcx + Vcy - Wcr * ( a + b ) ;
	Speed_Motor_Target_4 =   Vcx + Vcy + Wcr * ( a + b ) ;
}

void Transmit_Speed(void)
{
			Target_1 = PID_velocity_realize_1(Speed_Motor_Target_1,1);
			Target_2 = PID_velocity_realize_1(Speed_Motor_Target_2,2);
			Target_3 = PID_velocity_realize_1(Speed_Motor_Target_3,3);
			Target_4 = PID_velocity_realize_1(Speed_Motor_Target_4,4);
//	 		Target_1 = pid_call_1(Position_Motor_Target_1,1);
			CAN_cmd_chassis(Target_1,Target_2,Target_3,Target_4);
}
