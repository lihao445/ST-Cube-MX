#include "tim_user.h"

fp32 latitude;

fp32 longitude;

/* 重定义fputc函数, printf函数最终会通过调用fputc输出字符串到串口 */
int fputc(int ch, FILE *f)
{
    while ((USART2->SR & 0X40) == 0);               /* 等待上一个字符发送完成 */

    USART2->DR = (uint8_t)ch;                       /* 将要发送的字符 ch 写入到DR寄存器 */
    return ch;
}


int32_t i;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM6)
	{

	
	}

}