#ifndef __TIM_USER_H_
#define __TIM_USER_H_
#include "tim.h"
#include "can_receive.h"
#include "remote_control.h"
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

void RC_VC_DATA(void);
void collect(int i);
void shoot(int i);
void cam(void);
void climb(void);


#endif
