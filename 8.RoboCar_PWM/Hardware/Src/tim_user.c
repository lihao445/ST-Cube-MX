#include "tim_user.h"


extern PS2_UserDataTypedef PStwo;
float Speed_Motor_Target_1,Speed_Motor_Target_2,Speed_Motor_Target_3,Speed_Motor_Target_4;


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{		
		if(htim->Instance == htim1.Instance)               //htim==(&htim1)
		{
			HAL_GPIO_WritePin(MOTOR1_IN1_GPIO_Port, MOTOR1_IN1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(MOTOR1_IN2_GPIO_Port, MOTOR1_IN2_Pin, GPIO_PIN_RESET);	
			
			if(PStwo.PSS_LY > 0)
			{
				HAL_GPIO_WritePin(MOTOR1_IN1_GPIO_Port, MOTOR1_IN1_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(MOTOR1_IN2_GPIO_Port, MOTOR1_IN2_Pin, GPIO_PIN_RESET);	
				
				Speed_Motor_Target_1 = 	PStwo.PSS_LY;
				Speed_Motor_Target_2 = 	PStwo.PSS_LY;
				Speed_Motor_Target_3 = 	PStwo.PSS_LY;
				Speed_Motor_Target_4 = 	PStwo.PSS_LY;
			}
			if(PStwo.PSS_LY < 0)
			{
				HAL_GPIO_WritePin(MOTOR1_IN1_GPIO_Port, MOTOR1_IN1_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(MOTOR1_IN2_GPIO_Port, MOTOR1_IN2_Pin, GPIO_PIN_SET);	
				
				Speed_Motor_Target_1 = 	-PStwo.PSS_LY;
				Speed_Motor_Target_2 = 	-PStwo.PSS_LY;
				Speed_Motor_Target_3 = 	-PStwo.PSS_LY;
				Speed_Motor_Target_4 = 	-PStwo.PSS_LY;
			}
			







			__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_4,Speed_Motor_Target_1);   //ARR = 1280-1
			__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_1,Speed_Motor_Target_2);			
			__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_1,Speed_Motor_Target_3);			
			__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_1,Speed_Motor_Target_4);			
		}
}
