/**    《遥控器舵轮控制》
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

Chassis_Speed_t absolute_chassis_speed;

fp32 Speed_Motor_Target[4];
fp32 Shoot_Motor_Target[2];
fp32 Claw_Motor_Target[1];
fp32 Conveyer_Motor_Target[1];

fp32 M2006_Target[4];
fp32 M3508_Target[8];

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM1)
	{
		
		/***************************************底盘运动****************************************************************/
		 absolute_chassis_speed.vx =  (fp32)rc_ctrl.rc.ch[2]; 
     absolute_chassis_speed.vy =  (fp32)rc_ctrl.rc.ch[3]; 
     absolute_chassis_speed.vw = -(fp32)rc_ctrl.rc.ch[4]; 

     absolute_chassis_speed.vx *= 6;
     absolute_chassis_speed.vy *= 6;
     absolute_chassis_speed.vw *= 5;
		
		Speed_Motor_Target[0] =   absolute_chassis_speed.vy + absolute_chassis_speed.vx + absolute_chassis_speed.vw;
		Speed_Motor_Target[1] = - absolute_chassis_speed.vy + absolute_chassis_speed.vx + absolute_chassis_speed.vw;
		Speed_Motor_Target[2] = - absolute_chassis_speed.vy - absolute_chassis_speed.vx + absolute_chassis_speed.vw;
		Speed_Motor_Target[3] =   absolute_chassis_speed.vy - absolute_chassis_speed.vx + absolute_chassis_speed.vw;
		
		
		
		
		M3508_Target[0] = PID_velocity_realize_1(Speed_Motor_Target[0], 1);
    M3508_Target[1] = PID_velocity_realize_1(Speed_Motor_Target[1], 2);
    M3508_Target[2] = PID_velocity_realize_1(Speed_Motor_Target[2], 3);
    M3508_Target[3] = PID_velocity_realize_1(Speed_Motor_Target[3], 4);

    CAN1_CMD_1(M3508_Target[0], M3508_Target[1], M3508_Target[2], M3508_Target[3]);
		
		
		
		if (rc_ctrl.rc.s[0] == 3 && rc_ctrl.rc.s[1] == 1) //爪子定位模式
		{
		/**********************************爪子****************************************************************/
			if(rc_ctrl.rc.ch[1] >= 180)
			{
				Claw_Motor_Target[0] = 8191.0f * 19.2f * 0.0f;
				M3508_Target[4] = pid_call_2(Claw_Motor_Target[0],1);
			}
			else if(rc_ctrl.rc.ch[1] <= -180)
			{
				Claw_Motor_Target[0] = 8191.0f * 19.2f * 0.47f;
				M3508_Target[4] = pid_call_2(Claw_Motor_Target[0],1);
			}
			
			if(rc_ctrl.rc.ch[0] >= 580)
			{
				  HAL_GPIO_WritePin(Claw_Catch_GPIO_Port, Claw_Catch_Pin, GPIO_PIN_SET);
			}			
			else if(rc_ctrl.rc.ch[0] <= -580)
			{
				  HAL_GPIO_WritePin(Claw_Catch_GPIO_Port, Claw_Catch_Pin, GPIO_PIN_RESET);
			}
		}
		
		if (rc_ctrl.rc.s[0] == 3 && rc_ctrl.rc.s[1] == 2) //底盘发射模式
		{
				/****************************************发射装置****************************************************************/
			Shoot_Motor_Target[0] = rc_ctrl.rc.ch[1] *13.67;
			Shoot_Motor_Target[1] = -rc_ctrl.rc.ch[1] *13.67;
		
			for(int i = 5;i < 7;i++)
			{
				M3508_Target[i] = PID_velocity_realize_2(Shoot_Motor_Target[i-5],i-4+1);
			}
			/****************************************传送装置****************************************************************/
			
			
			
			if(rc_ctrl.rc.ch[0] >= 580)
			{
				Conveyer_Motor_Target[0] = 3000;
			}
			else if(rc_ctrl.rc.ch[0] <= -580)
			{
				Conveyer_Motor_Target[0] = 0;
			}
			M3508_Target[7] = PID_velocity_realize_2(Conveyer_Motor_Target[0],4);
			
		}
			CAN2_CMD_1(M3508_Target[4], M3508_Target[5], M3508_Target[6], M3508_Target[7]);   //CAN2的1,2,3,4号电机
		
		
		
			
		
		
		
		
		
	}
}
