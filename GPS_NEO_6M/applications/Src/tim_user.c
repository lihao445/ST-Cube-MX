#include "tim_user.h"

fp32 latitude;

fp32 longitude;

/* �ض���fputc����, printf�������ջ�ͨ������fputc����ַ��������� */
int fputc(int ch, FILE *f)
{
    while ((USART2->SR & 0X40) == 0);               /* �ȴ���һ���ַ�������� */

    USART2->DR = (uint8_t)ch;                       /* ��Ҫ���͵��ַ� ch д�뵽DR�Ĵ��� */
    return ch;
}


int32_t i;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM6)
	{

	
	}

}