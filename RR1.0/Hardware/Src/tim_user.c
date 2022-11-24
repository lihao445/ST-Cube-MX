#include "tim_user.h"
#include "struct_typedef.h"

float Vcx,Vcy,Vcr;

extern RC_ctrl_t rc_ctrl;

extern float Target_1,Target_2,Target_3,Target_4,Target_5,Target_6,Target_7,Target_8;
extern float Speed_Motor_Target_1,Speed_Motor_Target_2,Speed_Motor_Target_3,Speed_Motor_Target_4,Speed_Motor_Target_5,Speed_Motor_Target_6,Speed_Motor_Target_7;
extern float	Position_Motor_Target_8,Position_Motor_Target_1,Position_Motor_Target_2,Position_Motor_Target_3,Position_Motor_Target_4;







void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{		
		if(htim->Instance == htim1.Instance)               //htim==(&htim1)
		{
//			Speed_Motor_Target_1 = 300;
//      Position_Motor_Target_1 = 8192 * 2 *19;
			
			RC_VC_DATA();         //将遥控器左边数据赋值
			
			if(rc_ctrl.rc.s[0] == 3)       //S1中间
			{
				
						
				if(rc_ctrl.rc.s[1] == 3)    //S2中间    收集模式
				{
					Speed_Motor_Target_1 = - Vcx + Vcy;								//左上开始，顺时针标号1,2,3,4
					Speed_Motor_Target_2 =   Vcx + Vcy;
					Speed_Motor_Target_3 = - Vcx + Vcy;
					Speed_Motor_Target_4 =   Vcx + Vcy;
					if(rc_ctrl.rc.ch[4] != 0)
					{
						Speed_Motor_Target_1 = -Vcr ;
						Speed_Motor_Target_2 =  Vcr ;
						Speed_Motor_Target_3 =  Vcr ;
						Speed_Motor_Target_4 = -Vcr ;
					}
					
					
					if(rc_ctrl.rc.ch[0] < -100)   //向左停止收集
					{
						collect(0);
					}
					if(rc_ctrl.rc.ch[0] > 100)   //向右开启收集（为-100 - 100时不修改当前状态）
					{
						collect(1);
					}
					
				if(rc_ctrl.rc.s[1] == 1)    //S2上面   发射模式
				{
					Speed_Motor_Target_1 = - Vcx + Vcy;								//左上开始，顺时针标号1,2,3,4
					Speed_Motor_Target_2 =   Vcx + Vcy;
					Speed_Motor_Target_3 = - Vcx + Vcy;
					Speed_Motor_Target_4 =   Vcx + Vcy;
					if(rc_ctrl.rc.ch[4] != 0)
					{
						Speed_Motor_Target_1 = -Vcr ;
						Speed_Motor_Target_2 =  Vcr ;
						Speed_Motor_Target_3 =  Vcr ;
						Speed_Motor_Target_4 = -Vcr ;
					}
					
					if(rc_ctrl.rc.ch[1] > 50)
					{
						shoot(1);
					}
					if(rc_ctrl.rc.ch[1] <= 0)
					{
						shoot(0);
					}
					
					if(rc_ctrl.rc.ch[0] > 50)   //向右钮一下   若需要边用凸轮边发射，需要斜着扭遥控器。
					{
					cam();
					}
					
				}
			}
				
		}
			
			
			
			Target_1 = PID_velocity_realize_1(Speed_Motor_Target_1,1);
			Target_2 = PID_velocity_realize_1(Speed_Motor_Target_2,2);
			Target_3 = PID_velocity_realize_1(Speed_Motor_Target_3,3);
			Target_4 = PID_velocity_realize_1(Speed_Motor_Target_4,4);
//	 		Target_1 = pid_call_1(Position_Motor_Target_1,1);
			CAN1_cmd_chassis_1(Target_1,Target_2,Target_3,Target_4);
			CAN1_cmd_chassis_2(Target_5,Target_6,Target_7,Target_8);
			
		}

}

void climb(void)   //主函数调用
{
	RC_VC_DATA();
	if(rc_ctrl.rc.s[0] == 3)       //S1中间
			{
				if(rc_ctrl.rc.s[1] == 1)    //S2上面   同发射模式
				{
					if(rc_ctrl.rc.ch[0] < -50)  //向左扭一下，机器人会进入越阶模式
					{
						HAL_GPIO_WritePin(GPIOI, GPIO_PIN_7|GPIO_PIN_6, GPIO_PIN_SET);      //前两轮的杆
						HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);    //左后轮的杆
						HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);   //右后轮的杆
						HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9|GPIO_PIN_11, GPIO_PIN_RESET);   //前轮收起
						Target_1 = pid_call_1(Speed_Motor_Target_1,1);
						Target_2 = pid_call_1(Speed_Motor_Target_2,2);
						Target_3 = pid_call_1(Speed_Motor_Target_3,3);
						Target_4 = pid_call_1(Speed_Motor_Target_4,4);
						CAN1_cmd_chassis_1(Target_1,Target_2,Target_3,Target_4);
						HAL_Delay(200);
						HAL_GPIO_WritePin(GPIOI, GPIO_PIN_7|GPIO_PIN_6, GPIO_PIN_RESET);      //前两轮的杆
						HAL_Delay(1100);
						HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13|GPIO_PIN_14, GPIO_PIN_RESET);  //后轮收起
						HAL_Delay(200);
						HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);    //左后轮的杆
						HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);   //右后轮的杆
						HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13|GPIO_PIN_9|GPIO_PIN_11|GPIO_PIN_14, GPIO_PIN_SET);   //底盘轮子全部恢复正常行驶状态
					}
				}
			}
}



void collect(int i)
{
	if(i != 0)
	{
		Speed_Motor_Target_5 = 6000;    //速度6000
	}
	else
	{
		Speed_Motor_Target_5 = 0;
	}
	Target_5 = PID_velocity_realize_1(Speed_Motor_Target_5,5);
}




void shoot(int i)
{
	Speed_Motor_Target_6 = 9.1 * rc_ctrl.rc.ch[1];    //速度最高6000  因为是斜着扭，所以速度不会达到6000.
	Speed_Motor_Target_7 = 9.1 * rc_ctrl.rc.ch[1];
	if(i == 0)
	{
		Speed_Motor_Target_6 = 0;
		Speed_Motor_Target_7 = 0;
	}
	Target_6 = PID_velocity_realize_1(Speed_Motor_Target_6,6);
	Target_7 = PID_velocity_realize_1(Speed_Motor_Target_7,7);
}



void cam(void)
{
	Position_Motor_Target_8 = 8192 * (3591 / 187) * 1;
	Target_8 = pid_call_2(Position_Motor_Target_8,1);
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
