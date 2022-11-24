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
			
			RC_VC_DATA();         //��ң����������ݸ�ֵ
			
			if(rc_ctrl.rc.s[0] == 3)       //S1�м�
			{
				
						
				if(rc_ctrl.rc.s[1] == 3)    //S2�м�    �ռ�ģʽ
				{
					Speed_Motor_Target_1 = - Vcx + Vcy;								//���Ͽ�ʼ��˳ʱ����1,2,3,4
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
					
					
					if(rc_ctrl.rc.ch[0] < -100)   //����ֹͣ�ռ�
					{
						collect(0);
					}
					if(rc_ctrl.rc.ch[0] > 100)   //���ҿ����ռ���Ϊ-100 - 100ʱ���޸ĵ�ǰ״̬��
					{
						collect(1);
					}
					
				if(rc_ctrl.rc.s[1] == 1)    //S2����   ����ģʽ
				{
					Speed_Motor_Target_1 = - Vcx + Vcy;								//���Ͽ�ʼ��˳ʱ����1,2,3,4
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
					
					if(rc_ctrl.rc.ch[0] > 50)   //����ťһ��   ����Ҫ����͹�ֱ߷��䣬��Ҫб��Ťң������
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

void climb(void)   //����������
{
	RC_VC_DATA();
	if(rc_ctrl.rc.s[0] == 3)       //S1�м�
			{
				if(rc_ctrl.rc.s[1] == 1)    //S2����   ͬ����ģʽ
				{
					if(rc_ctrl.rc.ch[0] < -50)  //����Ťһ�£������˻����Խ��ģʽ
					{
						HAL_GPIO_WritePin(GPIOI, GPIO_PIN_7|GPIO_PIN_6, GPIO_PIN_SET);      //ǰ���ֵĸ�
						HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);    //����ֵĸ�
						HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);   //�Һ��ֵĸ�
						HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9|GPIO_PIN_11, GPIO_PIN_RESET);   //ǰ������
						Target_1 = pid_call_1(Speed_Motor_Target_1,1);
						Target_2 = pid_call_1(Speed_Motor_Target_2,2);
						Target_3 = pid_call_1(Speed_Motor_Target_3,3);
						Target_4 = pid_call_1(Speed_Motor_Target_4,4);
						CAN1_cmd_chassis_1(Target_1,Target_2,Target_3,Target_4);
						HAL_Delay(200);
						HAL_GPIO_WritePin(GPIOI, GPIO_PIN_7|GPIO_PIN_6, GPIO_PIN_RESET);      //ǰ���ֵĸ�
						HAL_Delay(1100);
						HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13|GPIO_PIN_14, GPIO_PIN_RESET);  //��������
						HAL_Delay(200);
						HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);    //����ֵĸ�
						HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);   //�Һ��ֵĸ�
						HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13|GPIO_PIN_9|GPIO_PIN_11|GPIO_PIN_14, GPIO_PIN_SET);   //��������ȫ���ָ�������ʻ״̬
					}
				}
			}
}



void collect(int i)
{
	if(i != 0)
	{
		Speed_Motor_Target_5 = 6000;    //�ٶ�6000
	}
	else
	{
		Speed_Motor_Target_5 = 0;
	}
	Target_5 = PID_velocity_realize_1(Speed_Motor_Target_5,5);
}




void shoot(int i)
{
	Speed_Motor_Target_6 = 9.1 * rc_ctrl.rc.ch[1];    //�ٶ����6000  ��Ϊ��б��Ť�������ٶȲ���ﵽ6000.
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

V1 =   Vcx + Vcy   (c��Chassis)
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
