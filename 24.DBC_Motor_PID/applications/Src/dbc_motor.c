#include "dbc_motor.h"

void Motor_CMD(uint32_t pulse)
{
	__HAL_TIM_SetCompare(&htim10,TIM_CHANNEL_1,pulse);
}
