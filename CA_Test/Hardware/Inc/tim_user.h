#ifndef __TIM_USER_H_
#define __TIM_USER_H_
#include "tim.h"
#include "can_receive.h"
#include "remote_control.h"
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

void RC_VC_DATA(void);

void Chassis_Stop(void);
void Chassis_Forward(float Chassis_Target);
void Chassis_Reverse(float Chassis_Target);
void Chassis_Left(float Chassis_Target);
void Chassis_Right(float Chassis_Target);
void Chassis_Ring_Left(float Chassis_Target);
void Chassis_Ring_Right(float Chassis_Target);

#endif
